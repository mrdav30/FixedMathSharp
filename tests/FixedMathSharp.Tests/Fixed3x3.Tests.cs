using System;
using MemoryPack;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests;

public class Fixed3x3Tests
{
    [Fact]
    public void CreateRotationX_WorksCorrectly()
    {
        var rotationMatrix = Fixed3x3.CreateRotationX(FixedMath.PiOver2); // 90 degrees
        var expectedMatrix = new Fixed3x3(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, -Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero
        );

        Assert.Equal(expectedMatrix, rotationMatrix);
    }

    [Fact]
    public void CreateRotationY_WorksCorrectly()
    {
        var rotationMatrix = Fixed3x3.CreateRotationY(FixedMath.PiOver2); // 90 degrees
        var expectedMatrix = new Fixed3x3(
            Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            -Fixed64.One, Fixed64.Zero, Fixed64.Zero
        );

        Assert.Equal(expectedMatrix, rotationMatrix);
    }

    [Fact]
    public void CreateRotationZ_WorksCorrectly()
    {
        var rotationMatrix = Fixed3x3.CreateRotationZ(FixedMath.PiOver2); // 90 degrees
        var expectedMatrix = new Fixed3x3(
            Fixed64.Zero, -Fixed64.One, Fixed64.Zero,
            Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );

        Assert.Equal(expectedMatrix, rotationMatrix);
    }

    [Fact]
    public void CreateShear_WorksCorrectly()
    {
        var shearMatrix = Fixed3x3.CreateShear(new Fixed64(1.0f), new Fixed64(0.5f), new Fixed64(0.2f));
        var expectedMatrix = new Fixed3x3(
            Fixed64.One, new Fixed64(1.0f), new Fixed64(0.5f),
            new Fixed64(1.0f), Fixed64.One, new Fixed64(0.2f),
            new Fixed64(0.5f), new Fixed64(0.2f), Fixed64.One
        );

        Assert.Equal(expectedMatrix, shearMatrix);
    }

    [Fact]
    public void InvertDiagonal_WorksCorrectly()
    {
        var matrix = new Fixed3x3(
            new Fixed64(2.0f), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, new Fixed64(3.0f), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, new Fixed64(4.0f)
        );

        var inverted = matrix.InvertDiagonal();
        var expected = new Fixed3x3(
            Fixed64.One / new Fixed64(2.0f), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One / new Fixed64(3.0f), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One / new Fixed64(4.0f)
        );

        Assert.Equal(expected, inverted);
    }

    [Fact]
    public void Lerp_WorksCorrectly()
    {
        var matrixA = Fixed3x3.Identity;
        var matrixB = new Fixed3x3(
            Fixed64.One, Fixed64.One, Fixed64.One,
            Fixed64.One, Fixed64.One, Fixed64.One,
            Fixed64.One, Fixed64.One, Fixed64.One
        );

        var result = Fixed3x3.Lerp(matrixA, matrixB, new Fixed64(0.5f));
        var expected = new Fixed3x3(
            Fixed64.One, new Fixed64(0.5f), new Fixed64(0.5f),
            new Fixed64(0.5f), Fixed64.One, new Fixed64(0.5f),
            new Fixed64(0.5f), new Fixed64(0.5f), Fixed64.One
        );

        Assert.Equal(expected, result);
    }

    [Fact]
    public void Transpose_WorksCorrectly()
    {
        var matrix = new Fixed3x3(
            Fixed64.One, new Fixed64(2.0f), new Fixed64(3.0f),
            new Fixed64(4.0f), Fixed64.One, new Fixed64(5.0f),
            new Fixed64(6.0f), new Fixed64(7.0f), Fixed64.One
        );

        var transposed = Fixed3x3.Transpose(matrix);
        var expected = new Fixed3x3(
            Fixed64.One, new Fixed64(4.0f), new Fixed64(6.0f),
            new Fixed64(2.0f), Fixed64.One, new Fixed64(7.0f),
            new Fixed64(3.0f), new Fixed64(5.0f), Fixed64.One
        );

        Assert.Equal(expected, transposed);
    }

    [Fact]
    public void Invert_WorksCorrectly()
    {
        var matrix = new Fixed3x3(
            new Fixed64(2.0f), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, new Fixed64(3.0f), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, new Fixed64(4.0f)
        );

        var success = Fixed3x3.Invert(matrix, out var result);
        Assert.True(success);

        var expected = new Fixed3x3(
            Fixed64.One / new Fixed64(2.0f), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One / new Fixed64(3.0f), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One / new Fixed64(4.0f)
        );

        Assert.True(result?.FuzzyEqual(expected), $"Expected: {expected}, Actual: {result}");
    }

    [Fact]
    public void Invert_SingularMatrix_ReturnsFalse()
    {
        var matrix = new Fixed3x3(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,  // Singular row
            Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );

        var success = Fixed3x3.Invert(matrix, out var _);
        Assert.False(success);
    }

    [Fact]
    public void Fixed3x3_Indexer_GetAndSet_UsesExpectedMapping()
    {
        var matrix = Fixed3x3.Zero;

        matrix[0] = new Fixed64(1);
        matrix[1] = new Fixed64(2);
        matrix[2] = new Fixed64(3);
        matrix[4] = new Fixed64(4);
        matrix[5] = new Fixed64(5);
        matrix[6] = new Fixed64(6);
        matrix[8] = new Fixed64(7);
        matrix[9] = new Fixed64(8);
        matrix[10] = new Fixed64(9);

        Assert.Equal(new Fixed64(1), matrix.m00);
        Assert.Equal(new Fixed64(2), matrix.m10);
        Assert.Equal(new Fixed64(3), matrix.m20);
        Assert.Equal(new Fixed64(4), matrix.m01);
        Assert.Equal(new Fixed64(5), matrix.m11);
        Assert.Equal(new Fixed64(6), matrix.m21);
        Assert.Equal(new Fixed64(7), matrix.m02);
        Assert.Equal(new Fixed64(8), matrix.m12);
        Assert.Equal(new Fixed64(9), matrix.m22);

        Assert.Equal(new Fixed64(1), matrix[0]);
        Assert.Equal(new Fixed64(2), matrix[1]);
        Assert.Equal(new Fixed64(3), matrix[2]);
        Assert.Equal(new Fixed64(4), matrix[4]);
        Assert.Equal(new Fixed64(5), matrix[5]);
        Assert.Equal(new Fixed64(6), matrix[6]);
        Assert.Equal(new Fixed64(7), matrix[8]);
        Assert.Equal(new Fixed64(8), matrix[9]);
        Assert.Equal(new Fixed64(9), matrix[10]);
    }

    [Fact]
    public void Fixed3x3_Indexer_InvalidIndex_Throws()
    {
        var matrix = Fixed3x3.Identity;

        Assert.Throws<IndexOutOfRangeException>(() => _ = matrix[-1]);
        Assert.Throws<IndexOutOfRangeException>(() => _ = matrix[3]);
        Assert.Throws<IndexOutOfRangeException>(() => _ = matrix[7]);
        Assert.Throws<IndexOutOfRangeException>(() => _ = matrix[11]);
        Assert.Throws<IndexOutOfRangeException>(() => matrix[-1] = Fixed64.One);
        Assert.Throws<IndexOutOfRangeException>(() => matrix[3] = Fixed64.One);
        Assert.Throws<IndexOutOfRangeException>(() => matrix[7] = Fixed64.One);
        Assert.Throws<IndexOutOfRangeException>(() => matrix[11] = Fixed64.One);
    }

    [Fact]
    public void Fixed3x3_SetLossyScale_Overloads_CreateExpectedMatrix()
    {
        var expected = new Fixed3x3(
            new Fixed64(2), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, new Fixed64(3), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, new Fixed64(4));

        Assert.Equal(expected, Fixed3x3.SetLossyScale(new Vector3d(2, 3, 4)));
        Assert.Equal(expected, Fixed3x3.SetLossyScale(new Fixed64(2), new Fixed64(3), new Fixed64(4)));
    }

    [Fact]
    public void InvertDiagonal_ZeroMiddleAxis_ReturnsOriginalMatrix()
    {
        var matrix = new Fixed3x3(
            new Fixed64(2), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, new Fixed64(4));

        Assert.Equal(matrix, matrix.InvertDiagonal());
    }

    [Fact]
    public void Fixed3x3_SetGlobalScale_WorksWithoutRotation()
    {
        var initialScale = new Vector3d(2, 2, 2);
        var globalScale = new Vector3d(4, 4, 4);

        var matrix = Fixed3x3.Identity;
        matrix.SetScale(initialScale);

        matrix.SetGlobalScale(globalScale);

        var extractedScale = Fixed3x3.ExtractScale(matrix);
        Assert.Equal(globalScale, extractedScale);
    }

    [Fact]
    public void Fixed3x3_SetGlobalScale_WorksWithRotation()
    {
        var rotationMatrix = new Fixed3x3(
            new Vector3d(0, 1, 0),   // Rotated X-axis
            new Vector3d(-1, 0, 0),  // Rotated Y-axis
            new Vector3d(0, 0, 1)    // Z-axis unchanged
        );
        var initialScale = new Vector3d(2, 2, 2);
        var globalScale = new Vector3d(4, 4, 4);

        // Apply initial scale
        rotationMatrix.SetScale(initialScale);

        // Set global scale
        rotationMatrix.SetGlobalScale(globalScale);

        // Extract final scale
        var extractedScale = Fixed3x3.ExtractLossyScale(rotationMatrix);

        Assert.True(
            extractedScale.FuzzyEqual(globalScale, new Fixed64(0.01)),
            $"Extracted scale {extractedScale} does not match expected {globalScale}."
        );
    }

    [Fact]
    public void Fixed3x3_ExtractScaleExtension_MatchesStaticImplementation()
    {
        var matrix = Fixed3x3.CreateScale(new Vector3d(2, 3, 4));

        Assert.Equal(Fixed3x3.ExtractScale(matrix), matrix.ExtractScale());
    }

    [Fact]
    public void Fixed3x3_FuzzyEqualExtensions_CoverTrueAndFalseBranches()
    {
        var baseline = new Fixed3x3(
            new Fixed64(1), new Fixed64(2), new Fixed64(3),
            new Fixed64(4), new Fixed64(5), new Fixed64(6),
            new Fixed64(7), new Fixed64(8), new Fixed64(9));
        var same = baseline;
        var changed = new Fixed3x3(
            new Fixed64(1), new Fixed64(2), new Fixed64(3),
            new Fixed64(4), new Fixed64(5), new Fixed64(6),
            new Fixed64(7), new Fixed64(8), new Fixed64(10));

        Assert.True(baseline.FuzzyEqualAbsolute(same, Fixed64.Zero));
        Assert.False(baseline.FuzzyEqualAbsolute(changed, new Fixed64(0.5)));
        Assert.True(baseline.FuzzyEqual(same));
        Assert.False(baseline.FuzzyEqual(changed, new Fixed64(0.01)));
    }

    [Fact]
    public void Fixed3x3_Normalize_WorksCorrectly()
    {
        var matrix = new Fixed3x3(
            new Vector3d(2, 0, 0),
            new Vector3d(0, 3, 0),
            new Vector3d(0, 0, 4)
        );

        matrix.Normalize();

        var xAxis = new Vector3d(matrix.m00, matrix.m01, matrix.m02);
        var yAxis = new Vector3d(matrix.m10, matrix.m11, matrix.m12);
        var zAxis = new Vector3d(matrix.m20, matrix.m21, matrix.m22);

        Assert.Equal(Fixed64.One, xAxis.Magnitude);
        Assert.Equal(Fixed64.One, yAxis.Magnitude);
        Assert.Equal(Fixed64.One, zAxis.Magnitude);
    }

    [Fact]
    public void TransformDirection_IdentityMatrix_ReturnsSameVector()
    {
        var matrix = Fixed3x3.Identity;
        var direction = new Vector3d(1, 2, 3);

        var transformed = Fixed3x3.TransformDirection(matrix, direction);

        Assert.Equal(direction, transformed);
    }

    [Fact]
    public void TransformDirection_90DegreeRotationX_WorksCorrectly()
    {
        var matrix = Fixed3x3.CreateRotationX(FixedMath.PiOver2); // 90-degree rotation around X-axis
        var direction = new Vector3d(0, 1, 0); // Pointing along Y-axis

        var transformed = Fixed3x3.TransformDirection(matrix, direction);

        // Expecting the direction to be rotated into the Z-axis
        var expected = new Vector3d(0, 0, 1);
        Assert.Equal(expected, transformed);
    }

    [Fact]
    public void TransformDirection_90DegreeRotationY_WorksCorrectly()
    {
        var matrix = Fixed3x3.CreateRotationY(FixedMath.PiOver2); // 90-degree rotation around Y-axis
        var direction = new Vector3d(1, 0, 0); // Pointing along X-axis

        var transformed = Fixed3x3.TransformDirection(matrix, direction);

        // Expecting the direction to be rotated into the negative Z-axis
        var expected = new Vector3d(0, 0, -1);
        Assert.Equal(expected, transformed);
    }

    [Fact]
    public void TransformDirection_90DegreeRotationZ_WorksCorrectly()
    {
        var matrix = Fixed3x3.CreateRotationZ(FixedMath.PiOver2); // 90-degree rotation around Z-axis
        var direction = new Vector3d(1, 0, 0); // Pointing along X-axis

        var transformed = Fixed3x3.TransformDirection(matrix, direction);

        // Expecting the direction to be rotated into the Y-axis
        var expected = new Vector3d(0, 1, 0);
        Assert.Equal(expected, transformed);
    }

    [Fact]
    public void TransformDirection_WithScaling_DirectionRemainsNormalized()
    {
        var matrix = Fixed3x3.CreateScale(new Vector3d(2, 3, 4));
        var direction = new Vector3d(1, 1, 1).Normal;

        var transformed = Fixed3x3.TransformDirection(matrix, direction).Normal;

        // Direction should still be normalized (scaling affects positions, not directions)
        Assert.Equal(Fixed64.One, transformed.Magnitude, new Fixed64(0.0001));
    }

    [Fact]
    public void InverseTransformDirection_IdentityMatrix_ReturnsSameVector()
    {
        var matrix = Fixed3x3.Identity;
        var direction = new Vector3d(1, 2, 3);

        var inverseTransformed = Fixed3x3.InverseTransformDirection(matrix, direction);

        Assert.Equal(direction, inverseTransformed);
    }

    [Fact]
    public void InverseTransformDirection_InvertsTransformDirection()
    {
        var matrix = Fixed3x3.CreateRotationY(FixedMath.PiOver2); // 90-degree Y-axis rotation
        var direction = new Vector3d(1, 0, 0);

        var transformed = Fixed3x3.TransformDirection(matrix, direction);
        var inverseTransformed = Fixed3x3.InverseTransformDirection(matrix, transformed);

        // The inverse should return the original direction
        Assert.Equal(direction, inverseTransformed);
    }

    [Fact]
    public void InverseTransformDirection_NonInvertibleMatrix_ThrowsException()
    {
        var singularMatrix = new Fixed3x3(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, // Singular row
            Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );

        var direction = new Vector3d(1, 1, 1);

        Assert.Throws<System.InvalidOperationException>(() =>
            Fixed3x3.InverseTransformDirection(singularMatrix, direction));
    }

    [Fact]
    public void Fixed3x3_OperatorsAndHashCode_WorkCorrectly()
    {
        var a = Fixed3x3.CreateScale(new Vector3d(2, 3, 4));
        var b = Fixed3x3.CreateScale(new Vector3d(5, 6, 7));

        Assert.Equal(
            new Fixed3x3(
                new Fixed64(7), Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, new Fixed64(9), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, new Fixed64(11)),
            a + b);
        Assert.Equal(
            new Fixed3x3(
                new Fixed64(-3), Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, new Fixed64(-3), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, new Fixed64(-3)),
            a - b);
        Assert.Equal(
            new Fixed3x3(
                new Fixed64(-2), Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, new Fixed64(-3), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, new Fixed64(-4)),
            -a);
        Assert.Equal(
            new Fixed3x3(
                new Fixed64(10), Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, new Fixed64(18), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, new Fixed64(28)),
            a * b);
        Assert.Equal(
            new Fixed3x3(
                new Fixed64(4), Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, new Fixed64(6), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, new Fixed64(8)),
            a * new Fixed64(2));
        Assert.Equal(
            new Fixed3x3(
                new Fixed64(4), Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, new Fixed64(6), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, new Fixed64(8)),
            new Fixed64(2) * a);
        Assert.Equal(
            new Fixed3x3(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, new Fixed64(1.5), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, new Fixed64(2)),
            a / 2);
        Assert.Equal(
            new Fixed3x3(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, new Fixed64(1.5), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, new Fixed64(2)),
            2 / a);

        var hash = a.GetHashCode();
        Assert.Equal(hash, Fixed3x3.CreateScale(new Vector3d(2, 3, 4)).GetHashCode());

        var changed = a;
        changed.m01 = Fixed64.One;
        Assert.NotEqual(hash, changed.GetHashCode());
    }

    #region Test: Serialization

    [Fact]
    public void Fixed3x3_NetSerialization_RoundTripMaintainsData()
    {
        var original3x3 = Fixed3x3.CreateRotationX(FixedMath.PiOver2); // 90 degrees

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(original3x3, jsonOptions);
        var deserialized3x3 = JsonSerializer.Deserialize<Fixed3x3>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(original3x3, deserialized3x3);
    }

    [Fact]
    public void Fixed3x3_MemoryPackSerialization_RoundTripMaintainsData()
    {
        Fixed3x3 originalValue = Fixed3x3.CreateRotationX(FixedMath.PiOver2); // 90 degrees

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Fixed3x3 deserializedValue = MemoryPackSerializer.Deserialize<Fixed3x3>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    #endregion
}
