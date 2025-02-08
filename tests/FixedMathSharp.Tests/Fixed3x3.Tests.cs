#if NET48_OR_GREATER
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
#endif

#if NET8_0_OR_GREATER
using System.Text.Json;
using System.Text.Json.Serialization;

#endif

using Xunit;

namespace FixedMathSharp.Tests
{
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

            var success = Fixed3x3.Invert(matrix, out var result);
            Assert.False(success);
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
        public void Fixed3x3_Serialization_RoundTripMaintainsData()
        {
            var original3x3 = Fixed3x3.CreateRotationX(FixedMath.PiOver2); // 90 degrees

            // Serialize the Fixed3x3 object
#if NET48_OR_GREATER
            var formatter = new BinaryFormatter();
            using var stream = new MemoryStream();
            formatter.Serialize(stream, original3x3);

            // Reset stream position and deserialize
            stream.Seek(0, SeekOrigin.Begin);
            var deserialized3x3 = (Fixed3x3)formatter.Deserialize(stream);
#endif

#if NET8_0_OR_GREATER
            var jsonOptions = new JsonSerializerOptions
            {
                DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
                ReferenceHandler = ReferenceHandler.IgnoreCycles,
                IncludeFields = true,
                IgnoreReadOnlyProperties = true
            };
            var json = JsonSerializer.SerializeToUtf8Bytes(original3x3, jsonOptions);
            var deserialized3x3 = JsonSerializer.Deserialize<Fixed3x3>(json, jsonOptions);
#endif

            // Check that deserialized values match the original
            Assert.Equal(original3x3, deserialized3x3);
        }
    }
}
