using System;
using MemoryPack;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests;

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
    public void FixedMatrix4x4_Decompose_LeftHandedMatrix_PreservesNegativeScale()
    {
        var translation = new Vector3d(1, 2, 3);
        var scale = new Vector3d(-2, 3, 4);
        var matrix = Fixed4x4.CreateTransform(translation, FixedQuaternion.Identity, scale);

        Assert.True(Fixed4x4.Decompose(matrix, out var decomposedScale, out var rotation, out var decomposedTranslation));

        Assert.Equal(scale, decomposedScale);
        Assert.Equal(translation, decomposedTranslation);
        Assert.True(rotation.FuzzyEqual(FixedQuaternion.Identity, new Fixed64(0.0001)));
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
        Assert.True(scale.FuzzyEqual(matrix.Scale, new Fixed64(0.0001)));
        Assert.True(matrix.Rotation.FuzzyEqual(rotation, new Fixed64(0.0001)),
            $"Extracted rotation {matrix.Rotation} does not match expected {rotation}.");
    }

    [Fact]
    public void FixedMatrix4x4_Indexer_GetAndSet_UsesExpectedMapping()
    {
        var matrix = Fixed4x4.Zero;

        for (int i = 0; i < 16; i++)
            matrix[i] = new Fixed64(i + 1);

        Assert.Equal(new Fixed64(1), matrix.m00);
        Assert.Equal(new Fixed64(2), matrix.m10);
        Assert.Equal(new Fixed64(3), matrix.m20);
        Assert.Equal(new Fixed64(4), matrix.m30);
        Assert.Equal(new Fixed64(5), matrix.m01);
        Assert.Equal(new Fixed64(6), matrix.m11);
        Assert.Equal(new Fixed64(7), matrix.m21);
        Assert.Equal(new Fixed64(8), matrix.m31);
        Assert.Equal(new Fixed64(9), matrix.m02);
        Assert.Equal(new Fixed64(10), matrix.m12);
        Assert.Equal(new Fixed64(11), matrix.m22);
        Assert.Equal(new Fixed64(12), matrix.m32);
        Assert.Equal(new Fixed64(13), matrix.m03);
        Assert.Equal(new Fixed64(14), matrix.m13);
        Assert.Equal(new Fixed64(15), matrix.m23);
        Assert.Equal(new Fixed64(16), matrix.m33);

        for (int i = 0; i < 16; i++)
            Assert.Equal(new Fixed64(i + 1), matrix[i]);
    }

    [Fact]
    public void FixedMatrix4x4_Indexer_InvalidIndex_Throws()
    {
        var matrix = Fixed4x4.Identity;

        Assert.Throws<IndexOutOfRangeException>(() => _ = matrix[-1]);
        Assert.Throws<IndexOutOfRangeException>(() => _ = matrix[16]);
        Assert.Throws<IndexOutOfRangeException>(() => matrix[-1] = Fixed64.One);
        Assert.Throws<IndexOutOfRangeException>(() => matrix[16] = Fixed64.One);
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
    public void FixedMatrix4x4_TranslateRotateScale_MatchesExplicitMultiplicationOrder()
    {
        var translation = new Vector3d(3, -2, 5);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)30, (Fixed64)45, (Fixed64)60);
        var scale = new Vector3d(2, 3, 4);

        var expected = Fixed4x4.CreateTranslation(translation) * Fixed4x4.CreateRotation(rotation) * Fixed4x4.CreateScale(scale);
        var result = Fixed4x4.TranslateRotateScale(translation, rotation, scale);

        Assert.Equal(expected, result);
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
    public void FixedMatrix4x4_SetScale_UpdatesDiagonalComponents()
    {
        var matrix = Fixed4x4.CreateTranslation(new Vector3d(5, 6, 7));

        var updated = Fixed4x4.SetScale(matrix, new Vector3d(2, 3, 4));

        Assert.Equal(new Fixed64(2), updated.m00);
        Assert.Equal(new Fixed64(3), updated.m11);
        Assert.Equal(new Fixed64(4), updated.m22);
        Assert.Equal(new Vector3d(5, 6, 7), updated.Translation);
    }

    [Fact]
    public void FixedMatrix4x4_SetRotation_PreservesTranslationAndScale()
    {
        var translation = new Vector3d(5, 6, 7);
        var scale = new Vector3d(2, 2, 2);
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, FixedMath.PiOver2);

        var matrix = Fixed4x4.CreateTransform(translation, FixedQuaternion.Identity, scale);
        var updated = Fixed4x4.SetRotation(matrix, rotation);

        Assert.Equal(translation, updated.Translation);
        Assert.True(scale.FuzzyEqual(updated.Scale, new Fixed64(0.0001)));
        Assert.True(updated.Rotation.FuzzyEqual(rotation, new Fixed64(0.0001)));
    }

    [Fact]
    public void FixedMatrix4x4_SetTranslationAndRotationExtensions_UpdateMatrixInPlace()
    {
        var matrix = Fixed4x4.CreateTransform(new Vector3d(1, 1, 1), FixedQuaternion.Identity, new Vector3d(2, 2, 2));
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2);

        matrix.SetTranslation(new Vector3d(7, 8, 9));
        var updated = matrix.SetRotation(rotation);

        Assert.Equal(new Vector3d(7, 8, 9), matrix.Translation);
        Assert.Equal(matrix, updated);
        Assert.True(new Vector3d(2, 2, 2).FuzzyEqual(matrix.Scale, new Fixed64(0.0001)));
        Assert.True(matrix.Rotation.FuzzyEqual(rotation, new Fixed64(0.0001)));
    }

    [Fact]
    public void FixedMatrix4x4_ExtractRotation_HandlesZeroScaleWithoutThrowing()
    {
        var matrix = Fixed4x4.CreateScale(Vector3d.Zero);

        var exception = Record.Exception(() => Fixed4x4.ExtractRotation(matrix));

        Assert.Null(exception);
    }

    [Fact]
    public void FixedMatrix4x4_Decompose_ZeroScaleMatrix_ReplacesZeroScaleToAvoidDivisionByZero()
    {
        var matrix = Fixed4x4.CreateScale(Vector3d.Zero);

        Assert.True(Fixed4x4.Decompose(matrix, out var scale, out var rotation, out var translation));

        Assert.Equal(Vector3d.One, scale);
        Assert.Equal(Vector3d.Zero, translation);
        Assert.Equal(rotation, rotation);
    }

    [Fact]
    public void FixedMatrix4x4_NormalizeRotationMatrixExtension_NormalizesAxesInPlace()
    {
        var matrix = new Fixed4x4(
            new Fixed64(2), Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, new Fixed64(3), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, new Fixed64(4), Fixed64.Zero,
            new Fixed64(5), new Fixed64(6), new Fixed64(7), Fixed64.One
        );

        matrix.NormalizeRotationMatrix();

        var xAxis = new Vector3d(matrix.m00, matrix.m01, matrix.m02);
        var yAxis = new Vector3d(matrix.m10, matrix.m11, matrix.m12);
        var zAxis = new Vector3d(matrix.m20, matrix.m21, matrix.m22);

        Assert.Equal(Fixed64.One, xAxis.Magnitude);
        Assert.Equal(Fixed64.One, yAxis.Magnitude);
        Assert.Equal(Fixed64.One, zAxis.Magnitude);
        Assert.Equal(Vector3d.Zero, matrix.Translation);
        Assert.Equal(Fixed64.One, matrix.m33);
    }

    [Fact]
    public void FixedMatrix4x4_Invert_NonAffineMatrix_WorksCorrectly()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );

        Assert.True(Fixed4x4.Invert(matrix, out var inverted));
        Assert.Equal(
            new Fixed4x4(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero, -Fixed64.One,
                Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            inverted);
        Assert.Equal(Fixed4x4.Identity, matrix * inverted);
    }

    [Fact]
    public void FixedMatrix4x4_Invert_SingularNonAffineMatrix_ReturnsFalse()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );

        Assert.False(Fixed4x4.Invert(matrix, out var inverted));
        Assert.Equal(Fixed4x4.Identity, inverted);
    }

    [Fact]
    public void FixedMatrix4x4_Invert_SingularAffineMatrix_ReturnsFalse()
    {
        var matrix = Fixed4x4.CreateScale(new Vector3d(0, 1, 1));

        Assert.False(Fixed4x4.Invert(matrix, out var inverted));
        Assert.Equal(Fixed4x4.Identity, inverted);
    }

    [Fact]
    public void FixedMatrix4x4_TransformPoint_NonAffineMatrix_UsesPerspectiveDivision()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );
        var point = new Vector3d(1, 2, 3);

        var transformed = Fixed4x4.TransformPoint(matrix, point);

        Assert.Equal(new Vector3d(new Fixed64(0.5), Fixed64.One, new Fixed64(1.5)), transformed);
    }

    [Fact]
    public void FixedMatrix4x4_TransformPoint_NonAffineMatrix_ZeroWFallsBackToOne()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, -Fixed64.One
        );

        var transformed = Fixed4x4.TransformPoint(matrix, new Vector3d(1, 2, 3));

        Assert.Equal(new Vector3d(2, 2, 3), transformed);
    }

    [Fact]
    public void FixedMatrix4x4_InverseTransformPoint_NonAffineMatrix_WorksCorrectly()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );
        var originalPoint = new Vector3d(1, 2, 3);
        var transformed = Fixed4x4.TransformPoint(matrix, originalPoint);

        var restored = Fixed4x4.InverseTransformPoint(matrix, transformed);

        Assert.True(originalPoint.FuzzyEqual(restored, new Fixed64(0.0001)));
    }

    [Fact]
    public void FixedMatrix4x4_InverseTransformPoint_NonAffineMatrix_ZeroWFallsBackToOne()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );

        var restored = Fixed4x4.InverseTransformPoint(matrix, new Vector3d(1, 2, 3));

        Assert.Equal(new Vector3d(1, 2, 3), restored);
    }

    [Fact]
    public void FixedMatrix4x4_InverseTransformPoint_NonInvertibleMatrix_Throws()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );

        Assert.Throws<InvalidOperationException>(() => Fixed4x4.InverseTransformPoint(matrix, Vector3d.Zero));
    }

    [Fact]
    public void FixedMatrix4x4_TransformPointExtensions_UseStaticImplementations()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );
        var point = new Vector3d(1, 2, 3);

        var transformed = matrix.TransformPoint(point);
        var restored = matrix.InverseTransformPoint(transformed);

        Assert.Equal(Fixed4x4.TransformPoint(matrix, point), transformed);
        Assert.True(point.FuzzyEqual(restored, new Fixed64(0.0001)));
    }

    [Fact]
    public void FixedMatrix4x4_OperatorsAndHashCode_WorkCorrectly()
    {
        var a = new Fixed4x4(
            new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4),
            new Fixed64(5), new Fixed64(6), new Fixed64(7), new Fixed64(8),
            new Fixed64(9), new Fixed64(10), new Fixed64(11), new Fixed64(12),
            new Fixed64(13), new Fixed64(14), new Fixed64(15), new Fixed64(16));
        var b = Fixed4x4.Identity;

        Assert.Equal(
            new Fixed4x4(
                new Fixed64(2), new Fixed64(2), new Fixed64(3), new Fixed64(4),
                new Fixed64(5), new Fixed64(7), new Fixed64(7), new Fixed64(8),
                new Fixed64(9), new Fixed64(10), new Fixed64(12), new Fixed64(12),
                new Fixed64(13), new Fixed64(14), new Fixed64(15), new Fixed64(17)),
            a + b);
        Assert.Equal(
            new Fixed4x4(
                Fixed64.Zero, new Fixed64(2), new Fixed64(3), new Fixed64(4),
                new Fixed64(5), new Fixed64(5), new Fixed64(7), new Fixed64(8),
                new Fixed64(9), new Fixed64(10), new Fixed64(10), new Fixed64(12),
                new Fixed64(13), new Fixed64(14), new Fixed64(15), new Fixed64(15)),
            a - b);
        Assert.Equal(
            new Fixed4x4(
                new Fixed64(-1), new Fixed64(-2), new Fixed64(-3), new Fixed64(-4),
                new Fixed64(-5), new Fixed64(-6), new Fixed64(-7), new Fixed64(-8),
                new Fixed64(-9), new Fixed64(-10), new Fixed64(-11), new Fixed64(-12),
                new Fixed64(-13), new Fixed64(-14), new Fixed64(-15), new Fixed64(-16)),
            -a);

        var hash = a.GetHashCode();
        Assert.Equal(hash, new Fixed4x4(
            new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4),
            new Fixed64(5), new Fixed64(6), new Fixed64(7), new Fixed64(8),
            new Fixed64(9), new Fixed64(10), new Fixed64(11), new Fixed64(12),
            new Fixed64(13), new Fixed64(14), new Fixed64(15), new Fixed64(16)).GetHashCode());

        var changedM03 = a;
        changedM03.m03 = new Fixed64(40);
        var changedM13 = a;
        changedM13.m13 = new Fixed64(80);

        Assert.NotEqual(hash, changedM03.GetHashCode());
        Assert.NotEqual(hash, changedM13.GetHashCode());
    }

    [Fact]
    public void FixedMatrix4x4_EqualsObject_ReturnsFalseForDifferentType()
    {
        Assert.False(Fixed4x4.Identity.Equals("not-a-matrix"));
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

    #region Test: Serialization

    [Fact]
    public void Fixed4x4_NetSerialization_RoundTripMaintainsData()
    {
        var translation = new Vector3d(1, 2, 3);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, FixedMath.PiOver2, Fixed64.Zero);
        var scale = new Vector3d(1, 1, 1);

        var original4x4 = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(original4x4, jsonOptions);
        var deserialized4x4 = JsonSerializer.Deserialize<Fixed4x4>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(original4x4, deserialized4x4);
    }

    [Fact]
    public void Fixed4x4_MemoryPackSerialization_RoundTripMaintainsData()
    {
        var translation = new Vector3d(1, 2, 3);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, FixedMath.PiOver2, Fixed64.Zero);
        var scale = new Vector3d(1, 1, 1);

        Fixed4x4 originalValue = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Fixed4x4 deserializedValue = MemoryPackSerializer.Deserialize<Fixed4x4>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    #endregion
}
