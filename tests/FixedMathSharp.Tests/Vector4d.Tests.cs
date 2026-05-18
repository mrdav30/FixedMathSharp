using MemoryPack;
using System;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests;

public class Vector4dTests
{
    [Fact]
    public void Constructor_Default_InitializesToZero()
    {
        var vector = new Vector4d();

        Assert.Equal(Fixed64.Zero, vector.x);
        Assert.Equal(Fixed64.Zero, vector.y);
        Assert.Equal(Fixed64.Zero, vector.z);
        Assert.Equal(Fixed64.Zero, vector.w);
    }

    [Fact]
    public void Constructor_Overloads_InitializeComponents()
    {
        var fromComponents = new Vector4d(Fixed64.One, new Fixed64(2), new Fixed64(3), new Fixed64(4));
        var fromScalar = new Vector4d(new Fixed64(5));
        var fromVector2 = new Vector4d(new Vector2d(1, 2), new Fixed64(3), new Fixed64(4));
        var fromVector3 = new Vector4d(new Vector3d(1, 2, 3), new Fixed64(4));

        Assert.Equal(new Vector4d(1, 2, 3, 4), fromComponents);
        Assert.Equal(new Vector4d(5, 5, 5, 5), fromScalar);
        Assert.Equal(new Vector4d(1, 2, 3, 4), fromVector2);
        Assert.Equal(new Vector4d(1, 2, 3, 4), fromVector3);
    }

    [Fact]
    public void StaticFields_ReturnExpectedValues()
    {
        Assert.Equal(new Vector4d(0, 0, 0, 0), Vector4d.Zero);
        Assert.Equal(new Vector4d(1, 1, 1, 1), Vector4d.One);
        Assert.Equal(new Vector4d(-1, -1, -1, -1), Vector4d.Negative);
        Assert.Equal(new Vector4d(1, 0, 0, 0), Vector4d.UnitX);
        Assert.Equal(new Vector4d(0, 1, 0, 0), Vector4d.UnitY);
        Assert.Equal(new Vector4d(0, 0, 1, 0), Vector4d.UnitZ);
        Assert.Equal(new Vector4d(0, 0, 0, 1), Vector4d.UnitW);
    }

    [Fact]
    public void Indexer_GetAndSet_RoundTripsComponents()
    {
        var vector = new Vector4d(1, 2, 3, 4);

        Assert.Equal(Fixed64.One, vector[0]);
        Assert.Equal(new Fixed64(2), vector[1]);
        Assert.Equal(new Fixed64(3), vector[2]);
        Assert.Equal(new Fixed64(4), vector[3]);

        vector[0] = new Fixed64(5);
        vector[1] = new Fixed64(6);
        vector[2] = new Fixed64(7);
        vector[3] = new Fixed64(8);

        Assert.Equal(new Vector4d(5, 6, 7, 8), vector);
    }

    [Fact]
    public void Indexer_InvalidIndex_Throws()
    {
        var vector = new Vector4d(1, 2, 3, 4);

        Assert.Throws<IndexOutOfRangeException>(() => _ = vector[-1]);
        Assert.Throws<IndexOutOfRangeException>(() => _ = vector[4]);
        Assert.Throws<IndexOutOfRangeException>(() => { vector[-1] = Fixed64.One; });
        Assert.Throws<IndexOutOfRangeException>(() => { vector[4] = Fixed64.One; });
    }

    [Fact]
    public void MagnitudeAndNormalization_WorkCorrectly()
    {
        var vector = new Vector4d(1, 2, 2, 4);

        Assert.Equal(new Fixed64(5), vector.Magnitude);
        Assert.Equal(new Fixed64(25), vector.SqrMagnitude);

        var normalized = vector.Normal;

        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.One, normalized.Magnitude);
    }

    [Fact]
    public void Normalize_ZeroVector_ReturnsZero()
    {
        var vector = Vector4d.Zero;

        Assert.Equal(Vector4d.Zero, vector.Normalize());
        Assert.Equal(Vector4d.Zero, Vector4d.GetNormalized(Vector4d.Zero));
    }

    [Fact]
    public void InPlaceHelpers_ModifyVectorCorrectly()
    {
        var vector = new Vector4d(1, 2, 3, 4);

        Assert.Equal(new Vector4d(5, 6, 7, 8), vector.AddInPlace(new Fixed64(4)));
        Assert.Equal(new Vector4d(6, 8, 10, 12), vector.AddInPlace(Fixed64.One, new Fixed64(2), new Fixed64(3), new Fixed64(4)));
        Assert.Equal(new Vector4d(7, 9, 11, 13), vector.AddInPlace(Vector4d.One));
        Assert.Equal(new Vector4d(6, 8, 10, 12), vector.SubtractInPlace(Fixed64.One));
        Assert.Equal(new Vector4d(4, 5, 6, 7), vector.SubtractInPlace(new Fixed64(2), new Fixed64(3), new Fixed64(4), new Fixed64(5)));
        Assert.Equal(new Vector4d(8, 15, 24, 35), vector.ScaleInPlace(new Vector4d(2, 3, 4, 5)));
    }

    [Fact]
    public void StaticHelpers_ReturnExpectedValues()
    {
        var a = new Vector4d(1, 2, 3, 4);
        var b = new Vector4d(5, 6, 7, 8);

        Assert.Equal(new Vector4d(3, 4, 5, 6), Vector4d.Lerp(a, b, Fixed64.Half));
        Assert.Equal(new Vector4d(-3, -2, -1, 0), Vector4d.UnclampedLerp(a, b, -Fixed64.One));
        Assert.Equal(new Fixed64(70), Vector4d.Dot(a, b));
        Assert.Equal(new Fixed64(8), Vector4d.SqrDistance(a, new Vector4d(3, 4, 3, 4)));
        Assert.Equal(new Vector4d(5, 12, 21, 32), Vector4d.Scale(a, b));
        Assert.Equal(new Vector4d(1, 2, 3, 4), Vector4d.Min(a, b));
        Assert.Equal(new Vector4d(5, 6, 7, 8), Vector4d.Max(a, b));
        Assert.Equal(new Vector4d(1, -1, 1, -1), Vector4d.Sign(new Vector4d(2, -3, 4, -5)));
        Assert.Equal(new Vector4d(1, 2, 7, 4), Vector4d.Clamp(new Vector4d(-1, 2, 9, 4), a, b));
    }

    [Fact]
    public void Operators_PerformComponentMath()
    {
        var a = new Vector4d(1, 2, 3, 4);
        var b = new Vector4d(5, 6, 7, 8);

        Assert.Equal(new Vector4d(6, 8, 10, 12), a + b);
        Assert.Equal(new Vector4d(4, 4, 4, 4), b - a);
        Assert.Equal(new Vector4d(-1, -2, -3, -4), -a);
        Assert.Equal(new Vector4d(2, 4, 6, 8), a * new Fixed64(2));
        Assert.Equal(new Vector4d(2, 4, 6, 8), new Fixed64(2) * a);
        Assert.Equal(new Vector4d(5, 12, 21, 32), a * b);
        Assert.Equal(new Vector4d(Fixed64.Half, Fixed64.One, new Fixed64(1.5), new Fixed64(2)), a / new Fixed64(2));
        Assert.Equal(new Vector4d(new Fixed64(5), new Fixed64(3), new Fixed64(7) / new Fixed64(3), new Fixed64(2)), b / a);
        Assert.True(a == new Vector4d(1, 2, 3, 4));
        Assert.True(a != b);
    }

    [Fact]
    public void Transform_WithTranslationMatrix_UsesHomogeneousW()
    {
        var matrix = Fixed4x4.CreateTranslation(new Fixed64(10), new Fixed64(20), new Fixed64(30));

        Assert.Equal(new Vector4d(11, 22, 33, 1), matrix * new Vector4d(1, 2, 3, 1));
        Assert.Equal(new Vector4d(1, 2, 3, 0), matrix * new Vector4d(1, 2, 3, 0));
    }

    [Fact]
    public void Transform_WithProjectionTerms_ComputesW()
    {
        var matrix = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        Assert.Equal(new Vector4d(2, 4, 6, 6), matrix * new Vector4d(2, 4, 6, 1));
    }

    [Fact]
    public void ConversionHelpers_ReturnExpectedValues()
    {
        var vector = new Vector4d(new Fixed64(1.25), new Fixed64(2.5), new Fixed64(3.75), new Fixed64(4.5));
        var vector2 = new Vector2d(new Fixed64(1.25), new Fixed64(2.5));
        var vector3 = new Vector3d(new Fixed64(1.25), new Fixed64(2.5), new Fixed64(3.75));

        float fx;
        float fy;
        float fz;
        float fw;
        vector.Deconstruct(out fx, out fy, out fz, out fw);

        int ix;
        int iy;
        int iz;
        int iw;
        vector.Deconstruct(out ix, out iy, out iz, out iw);

        Assert.Equal(new Vector3d(new Fixed64(1.25), new Fixed64(2.5), new Fixed64(3.75)), vector.ToVector3d());
        Assert.Equal(vector, vector2.ToVector4d(new Fixed64(3.75), new Fixed64(4.5)));
        Assert.Equal(vector, vector3.ToVector4d(new Fixed64(4.5)));
        Assert.Equal(1.25f, fx);
        Assert.Equal(2.5f, fy);
        Assert.Equal(3.75f, fz);
        Assert.Equal(4.5f, fw);
        Assert.Equal(1, ix);
        Assert.Equal(2, iy);
        Assert.Equal(4, iz);
        Assert.Equal(4, iw);
    }

    [Fact]
    public void FuzzyEqual_ComparesComponents()
    {
        var vector = new Vector4d(100, 100, 100, 100);
        var near = new Vector4d(100.0000008537, 100.0000008537, 100.0000008537, 100.0000008537);
        var far = new Vector4d(100.0001, 100.0001, 100.0001, 100.0001);

        Assert.True(vector.FuzzyEqual(near));
        Assert.False(vector.FuzzyEqual(far));
    }

    [Fact]
    public void Vector4d_NetSerialization_RoundTripMaintainsData()
    {
        var originalValue = new Vector4d(FixedMath.PI, FixedMath.PiOver2, FixedMath.TwoPI, Fixed64.Half);

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<Vector4d>(json, jsonOptions);

        Assert.Equal(originalValue, deserializedValue);
    }

    [Fact]
    public void Vector4d_MemoryPackSerialization_RoundTripMaintainsData()
    {
        Vector4d originalValue = new(FixedMath.PI, FixedMath.PiOver2, FixedMath.TwoPI, Fixed64.Half);

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Vector4d deserializedValue = MemoryPackSerializer.Deserialize<Vector4d>(bytes);

        Assert.Equal(originalValue, deserializedValue);
    }
}
