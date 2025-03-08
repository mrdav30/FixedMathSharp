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
    public class Fixed4x4Tests
    {
        [Fact]
        public void FixedMatrix4x4_FromMatrix_ConversionWorksCorrectly()
        {
            // Create a quaternion representing a 90-degree rotation around the Y-axis
            var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2); // 90 degrees

            // Create a rotation matrix using the quaternion
            var matrix = Fixed4x4.CreateRotation(rotation);

            // Extract the rotation back from the matrix
            var extractedRotation = FixedQuaternion.FromMatrix(matrix);

            // Check if the extracted rotation matches the original quaternion
            Assert.True(extractedRotation.FuzzyEqual(rotation, new Fixed64(0.0001)),
                $"Extracted rotation {extractedRotation} does not match expected {rotation}.");
        }

        [Fact]
        public void FixedMatrix4x4_CreateTranslation_WorksCorrectly()
        {
            var translation = new Vector3d(3, 4, 5);
            var matrix = Fixed4x4.CreateTranslation(translation);

            // Extract the translation to verify
            Assert.Equal(translation, matrix.Translation);
        }

        [Fact]
        public void FixedMatrix4x4_CreateScale_WorksCorrectly()
        {
            var scale = new Vector3d(2, 3, 4);
            var matrix = Fixed4x4.CreateScale(scale);

            // Extract the scale to verify
            Assert.Equal(scale, matrix.Scale);
        }

        [Fact]
        public void FixedMatrix4x4_Decompose_WorksCorrectly()
        {
            var matrix = Fixed4x4.Identity;
            matrix.m30 = new Fixed64(5);  // Add translation
            matrix.m00 = new Fixed64(2);  // Add scaling

            Assert.True(Fixed4x4.Decompose(
                matrix,
                out var scale,
                out var rotation,
                out var translation));

            Assert.Equal(new Vector3d(2, 1, 1), scale);
            Assert.Equal(new Vector3d(5, 0, 0), translation);
            Assert.Equal(FixedQuaternion.Identity, rotation);
        }

        [Fact]
        public void FixedMatrix4x4_SetTransform_WorksCorrectly()
        {
            var translation = new Vector3d(1, 2, 3);
            var scale = new Vector3d(2, 2, 2);
            var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2); // 90 degrees around Y-axis

            var matrix = Fixed4x4.Identity;
            matrix.SetTransform(translation, rotation, scale);

            // Extract and validate translation, scale, and rotation
            Assert.Equal(translation, matrix.Translation);
            Assert.Equal(scale, matrix.Scale);
            Assert.True(matrix.Rotation.FuzzyEqual(rotation, new Fixed64(0.0001)),
                $"Extracted rotation {matrix.Rotation} does not match expected {rotation}.");
        }

        [Fact]
        public void FixedMatrix4x4_LossyScale_NoRotation_WorksCorrectly()
        {
            var scale = new Vector3d(2, 2, 2);
            var matrix = Fixed4x4.Identity;
            matrix.SetTransform(new Vector3d(0, 0, 0), FixedQuaternion.Identity, scale);

            var extractedLossyScale = matrix.ExtractLossyScale();

            Assert.Equal(scale, extractedLossyScale);  // This should pass without rotation involved
        }

        [Fact]
        public void FixedMatrix4x4_LossyScale_NonUniformScale_WorksCorrectly()
        {
            var scale = new Vector3d(2, 3, 4);
            var matrix = Fixed4x4.Identity;
            matrix.SetTransform(new Vector3d(0, 0, 0), FixedQuaternion.Identity, scale);

            var extractedLossyScale = matrix.ExtractLossyScale();

            Assert.Equal(scale, extractedLossyScale);
        }

        [Fact]
        public void FixedMatrix4x4_Identity_IsCorrect()
        {
            var identity = Fixed4x4.Identity;
            Assert.Equal(Fixed64.One, identity.m00);
            Assert.Equal(Fixed64.One, identity.m11);
            Assert.Equal(Fixed64.One, identity.m22);
            Assert.Equal(Fixed64.One, identity.m33);

            Assert.All(new[]
            {
            identity.m01, identity.m02, identity.m03,
            identity.m10, identity.m12, identity.m13,
            identity.m20, identity.m21, identity.m23,
            identity.m30, identity.m31, identity.m32
        }, value => Assert.Equal(Fixed64.Zero, value));
        }

        [Fact]
        public void FixedMatrix4x4_Initialization_WorksCorrectly()
        {
            var matrix = new Fixed4x4(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );
            Assert.Equal(Fixed4x4.Identity, matrix);
        }

        [Fact]
        public void FixedMatrix4x4_GetDeterminant_WorksCorrectly()
        {
            var matrix = Fixed4x4.Identity;
            Assert.Equal(Fixed64.One, matrix.GetDeterminant());

            matrix = new Fixed4x4(
                Fixed64.One, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.One, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One,
                Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );
            Assert.Equal(Fixed64.Zero, matrix.GetDeterminant());

            matrix = new Fixed4x4(
                Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );

            Assert.Equal(new Fixed64(-1), matrix.GetDeterminant());
        }

        [Fact]
        public void FixedMatrix4x4_Invert_WorksCorrectly()
        {
            var matrix = new Fixed4x4(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );

            Assert.True(Fixed4x4.Invert(matrix, out var inverted));
            Assert.Equal(Fixed4x4.Identity, inverted);
        }

        [Fact]
        public void FixedMatrix4x4_Multiplication_WorksCorrectly()
        {
            var matrixA = new Fixed4x4(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );

            var matrixB = new Fixed4x4(
                Fixed64.One, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.One, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One,
                Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );

            var result = matrixA * matrixB;
            Assert.Equal(matrixB, result);
        }

        [Fact]
        public void FixedMatrix4x4_TRS_CreatesCorrectTransformationMatrix()
        {
            var translation = new Vector3d(3, -2, 5);
            var rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)30, (Fixed64)45, (Fixed64)60);
            var scale = new Vector3d(2, 3, 4);

            var trsMatrix = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

            // Instead of direct equality, compare the decomposed components
            Assert.True(Fixed4x4.Decompose(trsMatrix, out var decomposedScale, out var decomposedRotation, out var decomposedTranslation));

            Assert.True(translation.FuzzyEqual(decomposedTranslation, new Fixed64(0.0001)));
            Assert.True(scale.FuzzyEqual(decomposedScale, new Fixed64(0.0001)));
            Assert.True(decomposedRotation.FuzzyEqual(rotation, new Fixed64(0.0001)),
                $"Expected {rotation} but got {decomposedRotation}");
        }

        [Fact]
        public void FixedMatrix4x4_Equality_WorksCorrectly()
        {
            var matrixA = Fixed4x4.Identity;
            var matrixB = Fixed4x4.Identity;

            Assert.True(matrixA == matrixB);
            Assert.False(matrixA != matrixB);
        }

        [Fact]
        public void FixedMatrix4x4_SetGlobalScale_WorksWithoutRotation()
        {
            var initialScale = new Vector3d(2, 2, 2);
            var globalScale = new Vector3d(4, 4, 4);

            var matrix = Fixed4x4.Identity;
            matrix.SetTransform(Vector3d.Zero, FixedQuaternion.Identity, initialScale);

            // Apply global scaling
            matrix.SetGlobalScale(globalScale);

            // Extract the final scale
            Assert.Equal(globalScale, matrix.Scale);
        }

        [Fact]
        public void FixedMatrix4x4_SetGlobalScale_WorksWithRotation()
        {
            var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2); // 90 degrees Y-axis
            var initialScale = new Vector3d(2, 2, 2);
            var globalScale = new Vector3d(4, 4, 4);

            var matrix = Fixed4x4.Identity;
            matrix.SetTransform(Vector3d.Zero, rotation, initialScale);

            // Apply global scaling
            matrix.SetGlobalScale(globalScale);

            // Extract the final scale using ExtractLossyScale
            var extractedScale = matrix.ExtractLossyScale();

            Assert.Equal(globalScale, extractedScale);
        }

        [Fact]
        public void TransformPoint_WorldToLocal_ReturnsCorrectResult()
        {
            var translation = new Vector3d(7, 12, -5);
            var rotation = FixedQuaternion.FromEulerAnglesInDegrees(-(Fixed64)20, (Fixed64)35, (Fixed64)50);
            var scale = new Vector3d(1, 2, 1.5);

            var transformMatrix = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

            var worldPoint = new Vector3d(10, 15, -2);
            var localPoint = Fixed4x4.InverseTransformPoint(transformMatrix, worldPoint);
            var transformedBack = Fixed4x4.TransformPoint(transformMatrix, localPoint);

            Assert.True(worldPoint.FuzzyEqual(transformedBack, new Fixed64(0.01)),
                $"Expected {worldPoint} but got {transformedBack}");
        }

        [Fact]
        public void InverseTransformPoint_LocalToWorld_ReturnsCorrectResult()
        {
            var translation = new Vector3d(-4, 1, 2.5);
            var rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)45, -(Fixed64)30, (Fixed64)90);
            var scale = new Vector3d(1.2, 0.8, 1.5);

            var transformMatrix = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

            var localPoint = new Vector3d(2, 3, -1);
            var worldPoint = Fixed4x4.TransformPoint(transformMatrix, localPoint);
            var inverseTransformedPoint = Fixed4x4.InverseTransformPoint(transformMatrix, worldPoint);

            Assert.True(localPoint.FuzzyEqual(inverseTransformedPoint, new Fixed64(0.0001)),
                $"Expected {localPoint} but got {inverseTransformedPoint}");
        }

        [Fact]
        public void TransformPoint_InverseTransformPoint_RoundTripConsistency()
        {
            var translation = new Vector3d(2, -4, 8);
            var rotation = FixedQuaternion.FromEulerAnglesInDegrees(-(Fixed64)45, (Fixed64)30, (Fixed64)90);
            var scale = new Vector3d(1.5, 2.5, 3.0);

            var transformMatrix = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

            var originalPoint = new Vector3d(3, 5, 7);
            var transformedPoint = Fixed4x4.TransformPoint(transformMatrix, originalPoint);
            var inverseTransformedPoint = Fixed4x4.InverseTransformPoint(transformMatrix, transformedPoint);

            Assert.True(originalPoint.FuzzyEqual(inverseTransformedPoint, new Fixed64(0.0001)),
                $"Expected {originalPoint} but got {inverseTransformedPoint}");
        }

        [Fact]
        public void Fixed4x4_Serialization_RoundTripMaintainsData()
        {
            var translation = new Vector3d(1, 2, 3);
            var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, FixedMath.PiOver2, Fixed64.Zero);
            var scale = new Vector3d(1, 1, 1);

            var original4x4 = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

            // Serialize the Fixed4x4 object
#if NET48_OR_GREATER
            var formatter = new BinaryFormatter();
            using var stream = new MemoryStream();
            formatter.Serialize(stream, original4x4);

            // Reset stream position and deserialize
            stream.Seek(0, SeekOrigin.Begin);
            var deserialized4x4 = (Fixed4x4)formatter.Deserialize(stream);
#endif

#if NET8_0_OR_GREATER
            var jsonOptions = new JsonSerializerOptions {
                DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
                ReferenceHandler = ReferenceHandler.IgnoreCycles,
                IncludeFields = true,
                IgnoreReadOnlyProperties = true
            };
            var json = JsonSerializer.SerializeToUtf8Bytes(original4x4, jsonOptions);
            var deserialized4x4 = JsonSerializer.Deserialize<Fixed4x4>(json, jsonOptions);
#endif

            // Check that deserialized values match the original
            Assert.Equal(original4x4, deserialized4x4);
        }
    }
}
