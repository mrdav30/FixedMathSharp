using MemoryPack;
using System;
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
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi); // 90 degrees

        // Create a rotation matrix using the quaternion
        var matrix = Fixed4x4.CreateRotation(rotation);

        // Extract the rotation back from the matrix
        var extractedRotation = FixedQuaternion.FromMatrix(matrix);

        // Check if the extracted rotation matches the original quaternion
        Assert.True(extractedRotation.FuzzyEqual(rotation, Fixed64.FromFloatPoint(0.0001)),
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
    public void FixedMatrix4x4_FromRows_MapsVectorRows()
    {
        var row0 = new Vector4d(1, 2, 3, 4);
        var row1 = new Vector4d(5, 6, 7, 8);
        var row2 = new Vector4d(9, 10, 11, 12);
        var row3 = new Vector4d(13, 14, 15, 16);

        var matrix = Fixed4x4.FromRows(row0, row1, row2, row3);

        Assert.Equal(row0.X, matrix.M11);
        Assert.Equal(row0.Y, matrix.M12);
        Assert.Equal(row0.Z, matrix.M13);
        Assert.Equal(row0.W, matrix.M14);
        Assert.Equal(row1.X, matrix.M21);
        Assert.Equal(row1.Y, matrix.M22);
        Assert.Equal(row1.Z, matrix.M23);
        Assert.Equal(row1.W, matrix.M24);
        Assert.Equal(row2.X, matrix.M31);
        Assert.Equal(row2.Y, matrix.M32);
        Assert.Equal(row2.Z, matrix.M33);
        Assert.Equal(row2.W, matrix.M34);
        Assert.Equal(row3.X, matrix.M41);
        Assert.Equal(row3.Y, matrix.M42);
        Assert.Equal(row3.Z, matrix.M43);
        Assert.Equal(row3.W, matrix.M44);
    }

    [Fact]
    public void FixedMatrix4x4_FromColumns_MapsVectorColumns()
    {
        var column0 = new Vector4d(1, 5, 9, 13);
        var column1 = new Vector4d(2, 6, 10, 14);
        var column2 = new Vector4d(3, 7, 11, 15);
        var column3 = new Vector4d(4, 8, 12, 16);

        var matrix = Fixed4x4.FromColumns(column0, column1, column2, column3);

        Assert.Equal(column0.X, matrix.M11);
        Assert.Equal(column0.Y, matrix.M21);
        Assert.Equal(column0.Z, matrix.M31);
        Assert.Equal(column0.W, matrix.M41);
        Assert.Equal(column1.X, matrix.M12);
        Assert.Equal(column1.Y, matrix.M22);
        Assert.Equal(column1.Z, matrix.M32);
        Assert.Equal(column1.W, matrix.M42);
        Assert.Equal(column2.X, matrix.M13);
        Assert.Equal(column2.Y, matrix.M23);
        Assert.Equal(column2.Z, matrix.M33);
        Assert.Equal(column2.W, matrix.M43);
        Assert.Equal(column3.X, matrix.M14);
        Assert.Equal(column3.Y, matrix.M24);
        Assert.Equal(column3.Z, matrix.M34);
        Assert.Equal(column3.W, matrix.M44);
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
    public void FixedMatrix4x4_CreateScale_OverloadsSetExpectedDiagonalComponents()
    {
        var uniform = Fixed4x4.CreateScale(new Fixed64(3));
        var nonUniform = Fixed4x4.CreateScale(new Fixed64(2), new Fixed64(3), new Fixed64(4));

        Assert.Equal(new Vector3d(3, 3, 3), uniform.Scale);
        Assert.Equal(new Vector3d(2, 3, 4), nonUniform.Scale);
        Assert.Equal(Fixed64.One, uniform.M44);
        Assert.Equal(Fixed64.One, nonUniform.M44);
    }

    [Fact]
    public void FixedMatrix4x4_DirectionProperties_ReturnNormalizedAxes()
    {
        var matrix = Fixed4x4.CreateScale(new Vector3d(2, 3, 4));

        Assert.Equal(Vector3d.Right, matrix.Right);
        Assert.Equal(Vector3d.Left, matrix.Left);
        Assert.Equal(Vector3d.Up, matrix.Up);
        Assert.Equal(Vector3d.Down, matrix.Down);
        Assert.Equal(Vector3d.Forward, matrix.Forward);
        Assert.Equal(Vector3d.Backward, matrix.Backward);
    }

    [Fact]
    public void FixedMatrix4x4_CreateRotationAxisFactories_RotateAroundExpectedAxes()
    {
        var tolerance = Fixed64.FromFloatPoint(0.0001);

        Assert.True(Fixed4x4.TransformPoint(Fixed4x4.CreateRotationX(Fixed64.HalfPi), Vector3d.Up)
            .FuzzyEqual(Vector3d.Forward, tolerance));
        Assert.True(Fixed4x4.TransformPoint(Fixed4x4.CreateRotationY(Fixed64.HalfPi), Vector3d.Forward)
            .FuzzyEqual(Vector3d.Right, tolerance));
        Assert.True(Fixed4x4.TransformPoint(Fixed4x4.CreateRotationZ(Fixed64.HalfPi), Vector3d.Right)
            .FuzzyEqual(Vector3d.Up, tolerance));
    }

    [Fact]
    public void FixedMatrix4x4_CreateFromAxisAngleAndEulerAngles_MatchQuaternionFactories()
    {
        Assert.Equal(
            Fixed4x4.CreateRotation(FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi)),
            Fixed4x4.CreateFromAxisAngle(Vector3d.Up, Fixed64.HalfPi));

        Assert.Equal(
            Fixed4x4.CreateRotation(FixedQuaternion.FromEulerAngles(Fixed64.PiOver4, Fixed64.PiOver4, Fixed64.PiOver4)),
            Fixed4x4.CreateFromEulerAngles(Fixed64.PiOver4, Fixed64.PiOver4, Fixed64.PiOver4));
    }

    [Fact]
    public void FixedMatrix4x4_CreateLookAt_DefaultForwardViewIsIdentity()
    {
        Fixed4x4 view = Fixed4x4.CreateLookAt(Vector3d.Zero, Vector3d.Forward, Vector3d.Up);

        Assert.Equal(Fixed4x4.Identity, view);
    }

    [Fact]
    public void FixedMatrix4x4_CreateLookAt_TransformsWorldPointIntoCameraSpace()
    {
        Fixed4x4 view = Fixed4x4.CreateLookAt(new Vector3d(10, 0, 0), new Vector3d(10, 0, 1), Vector3d.Up);

        Vector3d result = Fixed4x4.TransformPoint(view, new Vector3d(10, 2, 3));

        Assert.Equal(new Vector3d(0, 2, 3), result);
    }

    [Fact]
    public void FixedMatrix4x4_CreateWorld_DefaultBasisPreservesPositionAndAxes()
    {
        Fixed4x4 world = Fixed4x4.CreateWorld(new Vector3d(1, 2, 3), Vector3d.Forward, Vector3d.Up);

        Assert.Equal(new Vector3d(1, 2, 3), world.Translation);
        Assert.Equal(Vector3d.Right, world.Right);
        Assert.Equal(Vector3d.Up, world.Up);
        Assert.Equal(Vector3d.Forward, world.Forward);
    }

    [Fact]
    public void FixedMatrix4x4_CreateOrthographic_MatchesIdentityFrustumForDefaultUnitVolume()
    {
        Fixed4x4 matrix = Fixed4x4.CreateOrthographic(new Fixed64(2), new Fixed64(2), Fixed64.Zero, Fixed64.One);

        Assert.Equal(Fixed4x4.Identity, matrix);
    }

    [Fact]
    public void FixedMatrix4x4_CreateOrthographicOffCenter_ProducesExpectedFrustum()
    {
        Fixed4x4 matrix = Fixed4x4.CreateOrthographicOffCenter(
            Fixed64.Zero,
            new Fixed64(2),
            -Fixed64.One,
            new Fixed64(3),
            Fixed64.One,
            new Fixed64(5));
        var frustum = new BoundingFrustum(matrix);

        Vector3d[] corners = frustum.GetCorners();

        Assert.True(corners[0].FuzzyEqual(new Vector3d(0, 3, 1), Fixed64.FromFloatPoint(0.0001)));
        Assert.True(corners[2].FuzzyEqual(new Vector3d(2, -1, 1), Fixed64.FromFloatPoint(0.0001)));
        Assert.True(corners[6].FuzzyEqual(new Vector3d(2, -1, 5), Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedMatrix4x4_CreatePerspective_MatchesFieldOfViewForEquivalentVolume()
    {
        Fixed4x4 perspective = Fixed4x4.CreatePerspective(new Fixed64(2), new Fixed64(2), Fixed64.One, new Fixed64(10));
        Fixed4x4 fieldOfView = Fixed4x4.CreatePerspectiveFieldOfView(Fixed64.HalfPi, Fixed64.One, Fixed64.One, new Fixed64(10));

        Assert.True(new BoundingFrustum(perspective).GetCorners()[6]
            .FuzzyEqual(new BoundingFrustum(fieldOfView).GetCorners()[6], Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedMatrix4x4_CreatePerspectiveFieldOfView_ProducesExpectedPositiveZFrustum()
    {
        Fixed4x4 matrix = Fixed4x4.CreatePerspectiveFieldOfView(Fixed64.HalfPi, Fixed64.One, Fixed64.One, new Fixed64(10));
        var frustum = new BoundingFrustum(matrix);

        Vector3d[] corners = frustum.GetCorners();

        Assert.True(corners[0].FuzzyEqual(new Vector3d(-1, 1, 1), Fixed64.FromFloatPoint(0.0001)));
        Assert.True(corners[6].FuzzyEqual(new Vector3d(10, -10, 10), Fixed64.FromFloatPoint(0.0001)));
        Assert.Equal(ContainmentType.Contains, frustum.Contains(new Vector3d(0, 0, 5)));
        Assert.Equal(ContainmentType.Disjoint, frustum.Contains(new Vector3d(0, 0, -1)));
        Assert.Equal(ContainmentType.Disjoint, frustum.Contains(new Vector3d(0, 0, 11)));
    }

    [Fact]
    public void FixedMatrix4x4_CreatePerspectiveOffCenter_ProducesExpectedFrustum()
    {
        Fixed4x4 matrix = Fixed4x4.CreatePerspectiveOffCenter(
            Fixed64.Zero,
            new Fixed64(2),
            -Fixed64.One,
            new Fixed64(3),
            Fixed64.One,
            new Fixed64(5));
        var frustum = new BoundingFrustum(matrix);

        Vector3d[] corners = frustum.GetCorners();

        Assert.True(corners[0].FuzzyEqual(new Vector3d(0, 3, 1), Fixed64.FromFloatPoint(0.0001)));
        Assert.True(corners[2].FuzzyEqual(new Vector3d(2, -1, 1), Fixed64.FromFloatPoint(0.0001)));
        Assert.True(corners[6].FuzzyEqual(new Vector3d(10, -5, 5), Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedMatrix4x4_CreatePerspectiveFieldOfView_InvalidArgumentsThrow()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspectiveFieldOfView(Fixed64.Zero, Fixed64.One, Fixed64.One, new Fixed64(10)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspectiveFieldOfView(Fixed64.Pi, Fixed64.One, Fixed64.One, new Fixed64(10)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspectiveFieldOfView(Fixed64.HalfPi, Fixed64.Zero, Fixed64.One, new Fixed64(10)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspectiveFieldOfView(Fixed64.HalfPi, Fixed64.One, Fixed64.Zero, new Fixed64(10)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspectiveFieldOfView(Fixed64.HalfPi, Fixed64.One, new Fixed64(10), Fixed64.One));
    }

    [Fact]
    public void FixedMatrix4x4_CreateLookAt_InvalidArgumentsThrow()
    {
        Assert.Throws<ArgumentException>(() =>
            Fixed4x4.CreateLookAt(Vector3d.Zero, Vector3d.Zero, Vector3d.Up));
        Assert.Throws<ArgumentException>(() =>
            Fixed4x4.CreateLookAt(Vector3d.Zero, Vector3d.Forward, Vector3d.Forward));
    }

    [Fact]
    public void FixedMatrix4x4_CreateWorld_InvalidArgumentsThrow()
    {
        Assert.Throws<ArgumentException>(() =>
            Fixed4x4.CreateWorld(Vector3d.Zero, Vector3d.Zero, Vector3d.Up));
        Assert.Throws<ArgumentException>(() =>
            Fixed4x4.CreateWorld(Vector3d.Zero, Vector3d.Forward, Vector3d.Forward));
    }

    [Fact]
    public void FixedMatrix4x4_CreateOrthographic_InvalidArgumentsThrow()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreateOrthographic(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreateOrthographic(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreateOrthographic(Fixed64.One, Fixed64.One, new Fixed64(-1), Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreateOrthographic(Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One));
    }

    [Fact]
    public void FixedMatrix4x4_CreateOrthographicOffCenter_InvalidArgumentsThrow()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreateOrthographicOffCenter(Fixed64.One, Fixed64.One, Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreateOrthographicOffCenter(Fixed64.Zero, Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.Zero, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreateOrthographicOffCenter(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One, new Fixed64(-1), Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreateOrthographicOffCenter(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One, Fixed64.One, Fixed64.One));
    }

    [Fact]
    public void FixedMatrix4x4_CreatePerspective_InvalidArgumentsThrow()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspective(Fixed64.Zero, Fixed64.One, Fixed64.One, new Fixed64(10)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspective(Fixed64.One, Fixed64.Zero, Fixed64.One, new Fixed64(10)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspective(Fixed64.One, Fixed64.One, Fixed64.Zero, new Fixed64(10)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspective(Fixed64.One, Fixed64.One, new Fixed64(10), new Fixed64(10)));
    }

    [Fact]
    public void FixedMatrix4x4_CreatePerspectiveOffCenter_InvalidArgumentsThrow()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspectiveOffCenter(Fixed64.One, Fixed64.One, Fixed64.Zero, Fixed64.One, Fixed64.One, new Fixed64(10)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspectiveOffCenter(Fixed64.Zero, Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One, new Fixed64(10)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspectiveOffCenter(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One, Fixed64.Zero, new Fixed64(10)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Fixed4x4.CreatePerspectiveOffCenter(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One, new Fixed64(10), new Fixed64(10)));
    }

    [Fact]
    public void FixedMatrix4x4_TransposeLerpAndScalarOperators_WorkComponentWise()
    {
        var matrix = new Fixed4x4(
            new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4),
            new Fixed64(5), new Fixed64(6), new Fixed64(7), new Fixed64(8),
            new Fixed64(9), new Fixed64(10), new Fixed64(11), new Fixed64(12),
            new Fixed64(13), new Fixed64(14), new Fixed64(15), new Fixed64(16));

        Assert.Equal(
            new Fixed4x4(
                new Fixed64(1), new Fixed64(5), new Fixed64(9), new Fixed64(13),
                new Fixed64(2), new Fixed64(6), new Fixed64(10), new Fixed64(14),
                new Fixed64(3), new Fixed64(7), new Fixed64(11), new Fixed64(15),
                new Fixed64(4), new Fixed64(8), new Fixed64(12), new Fixed64(16)),
            Fixed4x4.Transpose(matrix));

        Assert.Equal(matrix / Fixed64.Two, Fixed4x4.Lerp(Fixed4x4.Zero, matrix, Fixed64.Half));
        Assert.Equal(matrix + matrix, matrix * Fixed64.Two);
        Assert.Equal(matrix + matrix, Fixed64.Two * matrix);
        Assert.Equal(matrix, (matrix * Fixed64.Two) / Fixed64.Two);
    }

    [Fact]
    public void FixedMatrix4x4_ComponentDivide_DividesComponents()
    {
        var dividend = new Fixed4x4(
            new Fixed64(2), new Fixed64(6), new Fixed64(12), new Fixed64(20),
            new Fixed64(30), new Fixed64(42), new Fixed64(56), new Fixed64(72),
            new Fixed64(90), new Fixed64(110), new Fixed64(132), new Fixed64(156),
            new Fixed64(182), new Fixed64(210), new Fixed64(240), new Fixed64(272));
        var divisor = new Fixed4x4(
            new Fixed64(2), new Fixed64(3), new Fixed64(4), new Fixed64(5),
            new Fixed64(6), new Fixed64(7), new Fixed64(8), new Fixed64(9),
            new Fixed64(10), new Fixed64(11), new Fixed64(12), new Fixed64(13),
            new Fixed64(14), new Fixed64(15), new Fixed64(16), new Fixed64(17));
        var expected = new Fixed4x4(
            new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4),
            new Fixed64(5), new Fixed64(6), new Fixed64(7), new Fixed64(8),
            new Fixed64(9), new Fixed64(10), new Fixed64(11), new Fixed64(12),
            new Fixed64(13), new Fixed64(14), new Fixed64(15), new Fixed64(16));

        Assert.Equal(expected, Fixed4x4.ComponentDivide(dividend, divisor));
    }

    [Fact]
    public void FixedMatrix4x4_ComponentDivide_ByZeroThrows()
    {
        var dividend = new Fixed4x4(
            new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4),
            new Fixed64(5), new Fixed64(6), new Fixed64(7), new Fixed64(8),
            new Fixed64(9), new Fixed64(10), new Fixed64(11), new Fixed64(12),
            new Fixed64(13), new Fixed64(14), new Fixed64(15), new Fixed64(16));
        var divisor = new Fixed4x4(
            Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One,
            Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One,
            Fixed64.One, Fixed64.One, Fixed64.Zero, Fixed64.One,
            Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One);

        Assert.Throws<DivideByZeroException>(() => Fixed4x4.ComponentDivide(dividend, divisor));
    }

    [Fact]
    public void FixedMatrix4x4_Divide_MultipliesByInverseDivisor()
    {
        var dividend = Fixed4x4.CreateScale(new Vector3d(8, 12, 24));
        var divisor = Fixed4x4.CreateScale(new Vector3d(2, 4, 8));

        var result = Fixed4x4.Divide(dividend, divisor);

        Assert.Equal(Fixed4x4.CreateScale(new Vector3d(4, 3, 3)), result);
    }

    [Fact]
    public void FixedMatrix4x4_Divide_NonInvertibleDivisorThrows()
    {
        Assert.Throws<InvalidOperationException>(() => Fixed4x4.Divide(Fixed4x4.Identity, Fixed4x4.Zero));
    }

    [Fact]
    public void FixedMatrix4x4_Decompose_WorksCorrectly()
    {
        var matrix = Fixed4x4.Identity;
        matrix.M41 = new Fixed64(5);  // Add translation
        matrix.M11 = new Fixed64(2);  // Add scaling

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
        Assert.True(rotation.FuzzyEqual(FixedQuaternion.Identity, Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedMatrix4x4_SetTransform_WorksCorrectly()
    {
        var translation = new Vector3d(1, 2, 3);
        var scale = new Vector3d(2, 2, 2);
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi); // 90 degrees around Y-axis

        var matrix = Fixed4x4.Identity;
        matrix.SetTransform(translation, rotation, scale);

        // Extract and validate translation, scale, and rotation
        Assert.Equal(translation, matrix.Translation);
        Assert.True(scale.FuzzyEqual(matrix.Scale, Fixed64.FromFloatPoint(0.0001)));
        Assert.True(matrix.Rotation.FuzzyEqual(rotation, Fixed64.FromFloatPoint(0.0001)),
            $"Extracted rotation {matrix.Rotation} does not match expected {rotation}.");
    }

    [Fact]
    public void FixedMatrix4x4_Indexer_GetAndSet_UsesExpectedMapping()
    {
        var matrix = Fixed4x4.Zero;

        for (int i = 0; i < 16; i++)
            matrix[i] = new Fixed64(i + 1);

        Assert.Equal(new Fixed64(1), matrix.M11);
        Assert.Equal(new Fixed64(2), matrix.M21);
        Assert.Equal(new Fixed64(3), matrix.M31);
        Assert.Equal(new Fixed64(4), matrix.M41);
        Assert.Equal(new Fixed64(5), matrix.M12);
        Assert.Equal(new Fixed64(6), matrix.M22);
        Assert.Equal(new Fixed64(7), matrix.M32);
        Assert.Equal(new Fixed64(8), matrix.M42);
        Assert.Equal(new Fixed64(9), matrix.M13);
        Assert.Equal(new Fixed64(10), matrix.M23);
        Assert.Equal(new Fixed64(11), matrix.M33);
        Assert.Equal(new Fixed64(12), matrix.M43);
        Assert.Equal(new Fixed64(13), matrix.M14);
        Assert.Equal(new Fixed64(14), matrix.M24);
        Assert.Equal(new Fixed64(15), matrix.M34);
        Assert.Equal(new Fixed64(16), matrix.M44);

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
        Assert.Equal(Fixed64.One, identity.M11);
        Assert.Equal(Fixed64.One, identity.M22);
        Assert.Equal(Fixed64.One, identity.M33);
        Assert.Equal(Fixed64.One, identity.M44);

        Assert.All(new[]
        {
        identity.M12, identity.M13, identity.M14,
        identity.M21, identity.M23, identity.M24,
        identity.M31, identity.M32, identity.M34,
        identity.M41, identity.M42, identity.M43
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

        Assert.True(translation.FuzzyEqual(decomposedTranslation, Fixed64.FromFloatPoint(0.0001)));
        Assert.True(scale.FuzzyEqual(decomposedScale, Fixed64.FromFloatPoint(0.0001)));
        Assert.True(decomposedRotation.FuzzyEqual(rotation, Fixed64.FromFloatPoint(0.0001)),
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
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi); // 90 degrees Y-axis
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

        Assert.Equal(new Fixed64(2), updated.M11);
        Assert.Equal(new Fixed64(3), updated.M22);
        Assert.Equal(new Fixed64(4), updated.M33);
        Assert.Equal(new Vector3d(5, 6, 7), updated.Translation);
    }

    [Fact]
    public void FixedMatrix4x4_SetRotation_PreservesTranslationAndScale()
    {
        var translation = new Vector3d(5, 6, 7);
        var scale = new Vector3d(2, 2, 2);
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.HalfPi);

        var matrix = Fixed4x4.CreateTransform(translation, FixedQuaternion.Identity, scale);
        var updated = Fixed4x4.SetRotation(matrix, rotation);

        Assert.Equal(translation, updated.Translation);
        Assert.True(scale.FuzzyEqual(updated.Scale, Fixed64.FromFloatPoint(0.0001)));
        Assert.True(updated.Rotation.FuzzyEqual(rotation, Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedMatrix4x4_SetTranslationAndRotationExtensions_UpdateMatrixInPlace()
    {
        var matrix = Fixed4x4.CreateTransform(new Vector3d(1, 1, 1), FixedQuaternion.Identity, new Vector3d(2, 2, 2));
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);

        matrix.SetTranslation(new Vector3d(7, 8, 9));
        var updated = matrix.SetRotation(rotation);

        Assert.Equal(new Vector3d(7, 8, 9), matrix.Translation);
        Assert.Equal(matrix, updated);
        Assert.True(new Vector3d(2, 2, 2).FuzzyEqual(matrix.Scale, Fixed64.FromFloatPoint(0.0001)));
        Assert.True(matrix.Rotation.FuzzyEqual(rotation, Fixed64.FromFloatPoint(0.0001)));
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

        var xAxis = new Vector3d(matrix.M11, matrix.M12, matrix.M13);
        var yAxis = new Vector3d(matrix.M21, matrix.M22, matrix.M23);
        var zAxis = new Vector3d(matrix.M31, matrix.M32, matrix.M33);

        Assert.Equal(Fixed64.One, xAxis.Magnitude);
        Assert.Equal(Fixed64.One, yAxis.Magnitude);
        Assert.Equal(Fixed64.One, zAxis.Magnitude);
        Assert.Equal(Vector3d.Zero, matrix.Translation);
        Assert.Equal(Fixed64.One, matrix.M44);
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

        Assert.Equal(new Vector3d(Fixed64.FromFloatPoint(0.5), Fixed64.One, Fixed64.FromFloatPoint(1.5)), transformed);
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

        Assert.True(originalPoint.FuzzyEqual(restored, Fixed64.FromFloatPoint(0.0001)));
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
        Assert.True(point.FuzzyEqual(restored, Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedMatrix4x4_TransformVector4dHelpers_UseVector4dTransform()
    {
        Fixed4x4 matrix = Fixed4x4.CreateTranslation(new Vector3d(10, 20, 30));
        var vector = new Vector4d(1, 2, 3, 1);

        var staticResult = Fixed4x4.Transform(matrix, vector);
        var extensionResult = matrix.Transform(vector);

        Assert.Equal(Vector4d.Transform(matrix, vector), staticResult);
        Assert.Equal(staticResult, extensionResult);
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
        changedM03.M14 = new Fixed64(40);
        var changedM13 = a;
        changedM13.M24 = new Fixed64(80);

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
        var scale = Vector3d.FromFloatPoint(1, 2, 1.5);

        var transformMatrix = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

        var worldPoint = new Vector3d(10, 15, -2);
        var localPoint = Fixed4x4.InverseTransformPoint(transformMatrix, worldPoint);
        var transformedBack = Fixed4x4.TransformPoint(transformMatrix, localPoint);

        Assert.True(worldPoint.FuzzyEqual(transformedBack, Fixed64.FromFloatPoint(0.01)),
            $"Expected {worldPoint} but got {transformedBack}");
    }

    [Fact]
    public void InverseTransformPoint_LocalToWorld_ReturnsCorrectResult()
    {
        var translation = Vector3d.FromFloatPoint(-4, 1, 2.5);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)45, -(Fixed64)30, (Fixed64)90);
        var scale = Vector3d.FromFloatPoint(1.2, 0.8, 1.5);

        var transformMatrix = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

        var localPoint = new Vector3d(2, 3, -1);
        var worldPoint = Fixed4x4.TransformPoint(transformMatrix, localPoint);
        var inverseTransformedPoint = Fixed4x4.InverseTransformPoint(transformMatrix, worldPoint);

        Assert.True(localPoint.FuzzyEqual(inverseTransformedPoint, Fixed64.FromFloatPoint(0.0001)),
            $"Expected {localPoint} but got {inverseTransformedPoint}");
    }

    [Fact]
    public void TransformPoint_InverseTransformPoint_RoundTripConsistency()
    {
        var translation = new Vector3d(2, -4, 8);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(-(Fixed64)45, (Fixed64)30, (Fixed64)90);
        var scale = Vector3d.FromFloatPoint(1.5, 2.5, 3.0);

        var transformMatrix = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

        var originalPoint = new Vector3d(3, 5, 7);
        var transformedPoint = Fixed4x4.TransformPoint(transformMatrix, originalPoint);
        var inverseTransformedPoint = Fixed4x4.InverseTransformPoint(transformMatrix, transformedPoint);

        Assert.True(originalPoint.FuzzyEqual(inverseTransformedPoint, Fixed64.FromFloatPoint(0.0001)),
            $"Expected {originalPoint} but got {inverseTransformedPoint}");
    }

    #region Test: Serialization

    [Fact]
    public void Fixed4x4_NetSerialization_RoundTripMaintainsData()
    {
        var translation = new Vector3d(1, 2, 3);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.HalfPi, Fixed64.Zero);
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

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void Fixed4x4_MemoryPackSerialization_RoundTripMaintainsData()
    {
        var translation = new Vector3d(1, 2, 3);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.HalfPi, Fixed64.Zero);
        var scale = new Vector3d(1, 1, 1);

        Fixed4x4 originalValue = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Fixed4x4 deserializedValue = MemoryPackSerializer.Deserialize<Fixed4x4>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }
#endif

    #endregion
}
