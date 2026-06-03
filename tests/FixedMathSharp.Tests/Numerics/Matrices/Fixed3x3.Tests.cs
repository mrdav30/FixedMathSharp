using MemoryPack;
using System;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests;

public class Fixed3x3Tests
{
    [Fact]
    public void CreateRotationX_WorksCorrectly()
    {
        var rotationMatrix = Fixed3x3.CreateRotationX(Fixed64.HalfPi); // 90 degrees
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
        var rotationMatrix = Fixed3x3.CreateRotationY(Fixed64.HalfPi); // 90 degrees
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
        var rotationMatrix = Fixed3x3.CreateRotationZ(Fixed64.HalfPi); // 90 degrees
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
        var shearMatrix = Fixed3x3.CreateShear(Fixed64.FromFloatPoint(1.0), Fixed64.FromFloatPoint(0.5), Fixed64.FromFloatPoint(0.2));
        var expectedMatrix = new Fixed3x3(
            Fixed64.One, Fixed64.FromFloatPoint(1.0), Fixed64.FromFloatPoint(0.5),
            Fixed64.FromFloatPoint(1.0), Fixed64.One, Fixed64.FromFloatPoint(0.2),
            Fixed64.FromFloatPoint(0.5), Fixed64.FromFloatPoint(0.2), Fixed64.One
        );

        Assert.Equal(expectedMatrix, shearMatrix);
    }

    [Fact]
    public void CreateScale_WithUniformFixed64_SetsAllDiagonalComponents()
    {
        var matrix = Fixed3x3.CreateScale(new Fixed64(3));

        Assert.Equal(
            new Fixed3x3(
                new Fixed64(3), Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, new Fixed64(3), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, new Fixed64(3)),
            matrix);
    }

    [Fact]
    public void InvertDiagonal_WorksCorrectly()
    {
        var matrix = new Fixed3x3(
            Fixed64.FromFloatPoint(2.0), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.FromFloatPoint(3.0), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.FromFloatPoint(4.0)
        );

        var inverted = matrix.InvertDiagonal();
        var expected = new Fixed3x3(
            Fixed64.One / Fixed64.FromFloatPoint(2.0), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One / Fixed64.FromFloatPoint(3.0), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One / Fixed64.FromFloatPoint(4.0)
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

        var result = Fixed3x3.Lerp(matrixA, matrixB, Fixed64.FromFloatPoint(0.5f));
        var expected = new Fixed3x3(
            Fixed64.One, Fixed64.FromFloatPoint(0.5f), Fixed64.FromFloatPoint(0.5f),
            Fixed64.FromFloatPoint(0.5f), Fixed64.One, Fixed64.FromFloatPoint(0.5f),
            Fixed64.FromFloatPoint(0.5f), Fixed64.FromFloatPoint(0.5f), Fixed64.One
        );

        Assert.Equal(expected, result);
    }

    [Fact]
    public void Transpose_WorksCorrectly()
    {
        var matrix = new Fixed3x3(
            Fixed64.One, Fixed64.FromFloatPoint(2.0f), Fixed64.FromFloatPoint(3.0f),
            Fixed64.FromFloatPoint(4.0f), Fixed64.One, Fixed64.FromFloatPoint(5.0f),
            Fixed64.FromFloatPoint(6.0f), Fixed64.FromFloatPoint(7.0f), Fixed64.One
        );

        var transposed = Fixed3x3.Transpose(matrix);
        var expected = new Fixed3x3(
            Fixed64.One, Fixed64.FromFloatPoint(4.0f), Fixed64.FromFloatPoint(6.0f),
            Fixed64.FromFloatPoint(2.0f), Fixed64.One, Fixed64.FromFloatPoint(7.0f),
            Fixed64.FromFloatPoint(3.0f), Fixed64.FromFloatPoint(5.0f), Fixed64.One
        );

        Assert.Equal(expected, transposed);
    }

    [Fact]
    public void Invert_WorksCorrectly()
    {
        var matrix = new Fixed3x3(
            Fixed64.FromFloatPoint(2.0f), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.FromFloatPoint(3.0f), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.FromFloatPoint(4.0f)
        );

        var success = Fixed3x3.Invert(matrix, out var result);
        Assert.True(success);

        var expected = new Fixed3x3(
            Fixed64.One / Fixed64.FromFloatPoint(2.0f), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One / Fixed64.FromFloatPoint(3.0f), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One / Fixed64.FromFloatPoint(4.0f)
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

        Assert.Equal(new Fixed64(1), matrix.M11);
        Assert.Equal(new Fixed64(2), matrix.M21);
        Assert.Equal(new Fixed64(3), matrix.M31);
        Assert.Equal(new Fixed64(4), matrix.M12);
        Assert.Equal(new Fixed64(5), matrix.M22);
        Assert.Equal(new Fixed64(6), matrix.M32);
        Assert.Equal(new Fixed64(7), matrix.M13);
        Assert.Equal(new Fixed64(8), matrix.M23);
        Assert.Equal(new Fixed64(9), matrix.M33);

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
            extractedScale.FuzzyEqual(globalScale, Fixed64.FromFloatPoint(0.01)),
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
        Assert.False(baseline.FuzzyEqualAbsolute(changed, Fixed64.FromFloatPoint(0.5)));
        Assert.True(baseline.FuzzyEqual(same));
        Assert.False(baseline.FuzzyEqual(changed, Fixed64.FromFloatPoint(0.01)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    public void Fixed3x3_FuzzyEqualAbsolute_ReturnsFalse_WhenAnyComponentExceedsTolerance(int componentIndex)
    {
        var baseline = CreateSequentialMatrix3x3();
        var changed = OffsetMatrixComponent(baseline, componentIndex, Fixed64.FromFloatPoint(0.2));

        Assert.False(baseline.FuzzyEqualAbsolute(changed, Fixed64.FromFloatPoint(0.1)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    public void Fixed3x3_FuzzyEqual_ReturnsFalse_WhenAnyComponentExceedsPercentage(int componentIndex)
    {
        var baseline = CreateSequentialMatrix3x3();
        var changed = OffsetMatrixComponent(baseline, componentIndex, new Fixed64(10));

        Assert.False(baseline.FuzzyEqual(changed, Fixed64.FromFloatPoint(0.01)));
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

        var xAxis = new Vector3d(matrix.M11, matrix.M12, matrix.M13);
        var yAxis = new Vector3d(matrix.M21, matrix.M22, matrix.M23);
        var zAxis = new Vector3d(matrix.M31, matrix.M32, matrix.M33);

        Assert.Equal(Fixed64.One, xAxis.Magnitude);
        Assert.Equal(Fixed64.One, yAxis.Magnitude);
        Assert.Equal(Fixed64.One, zAxis.Magnitude);
    }

    [Fact]
    public void Fixed3x3_InvertDiagonal_LeavesZeroOuterDiagonalEntriesAtZero()
    {
        var matrix = new Fixed3x3(
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, new Fixed64(2), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        Assert.Equal(
            new Fixed3x3(
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Half, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            matrix.InvertDiagonal());
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
        var matrix = Fixed3x3.CreateRotationX(Fixed64.HalfPi); // 90-degree rotation around X-axis
        var direction = new Vector3d(0, 1, 0); // Pointing along Y-axis

        var transformed = Fixed3x3.TransformDirection(matrix, direction);

        // Expecting the direction to be rotated into the Z-axis
        var expected = new Vector3d(0, 0, 1);
        Assert.Equal(expected, transformed);
    }

    [Fact]
    public void TransformDirection_90DegreeRotationY_WorksCorrectly()
    {
        var matrix = Fixed3x3.CreateRotationY(Fixed64.HalfPi); // 90-degree rotation around Y-axis
        var direction = new Vector3d(1, 0, 0); // Pointing along X-axis

        var transformed = Fixed3x3.TransformDirection(matrix, direction);

        // Expecting the direction to be rotated into the negative Z-axis
        var expected = new Vector3d(0, 0, -1);
        Assert.Equal(expected, transformed);
    }

    [Fact]
    public void TransformDirection_90DegreeRotationZ_WorksCorrectly()
    {
        var matrix = Fixed3x3.CreateRotationZ(Fixed64.HalfPi); // 90-degree rotation around Z-axis
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
        Assert.Equal(Fixed64.One, transformed.Magnitude, Fixed64.FromFloatPoint(0.0001));
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
        var matrix = Fixed3x3.CreateRotationY(Fixed64.HalfPi); // 90-degree Y-axis rotation
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

        Assert.Throws<InvalidOperationException>(() =>
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
                Fixed64.Zero, Fixed64.FromFloatPoint(1.5), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, new Fixed64(2)),
            a / 2);

        var hash = a.GetHashCode();
        Assert.Equal(hash, Fixed3x3.CreateScale(new Vector3d(2, 3, 4)).GetHashCode());

        var changed = a;
        changed.M12 = Fixed64.One;
        Assert.True(a != changed);
        Assert.NotEqual(hash, changed.GetHashCode());
        Assert.False(a.Equals((object)"not-a-matrix"));
    }

    #region Test: Serialization

    [Fact]
    public void Fixed3x3_NetSerialization_RoundTripMaintainsData()
    {
        var original3x3 = Fixed3x3.CreateRotationX(Fixed64.HalfPi); // 90 degrees

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

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void Fixed3x3_MemoryPackSerialization_RoundTripMaintainsData()
    {
        Fixed3x3 originalValue = Fixed3x3.CreateRotationX(Fixed64.HalfPi); // 90 degrees

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Fixed3x3 deserializedValue = MemoryPackSerializer.Deserialize<Fixed3x3>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }
#endif

    #endregion

    private static Fixed3x3 CreateSequentialMatrix3x3()
    {
        return new Fixed3x3(
            new Fixed64(1), new Fixed64(2), new Fixed64(3),
            new Fixed64(4), new Fixed64(5), new Fixed64(6),
            new Fixed64(7), new Fixed64(8), new Fixed64(9));
    }

    private static Fixed3x3 OffsetMatrixComponent(Fixed3x3 matrix, int componentIndex, Fixed64 offset)
    {
        switch (componentIndex)
        {
            case 0:
                matrix.M11 += offset;
                break;
            case 1:
                matrix.M12 += offset;
                break;
            case 2:
                matrix.M13 += offset;
                break;
            case 3:
                matrix.M21 += offset;
                break;
            case 4:
                matrix.M22 += offset;
                break;
            case 5:
                matrix.M23 += offset;
                break;
            case 6:
                matrix.M31 += offset;
                break;
            case 7:
                matrix.M32 += offset;
                break;
            case 8:
                matrix.M33 += offset;
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(componentIndex));
        }

        return matrix;
    }
}
