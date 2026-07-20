using Xunit;

namespace FixedMathSharp.Tests;

public sealed class NormalizationContractTests
{
    private const long NearNegativeUnitRaw = -4_294_967_445L;
    private const long NearPositiveUnitRaw = 4_294_967_445L;
    private const long MinorComponentRaw = 150L;

    [Fact]
    public void OrdinaryMagnitudeDivision_RepairsBoundaryRoundingAcrossEveryDimension()
    {
        Fixed64 component2d = Fixed64.FromRaw(189_813_442L);
        var vector2 = new Vector2d(component2d, component2d);
        Assert.True(RequiresWideRepair(vector2));
        Vector2d normalized2 = vector2.Normalized;
        Vector2d inPlace2 = vector2;
        Assert.True(normalized2.IsNormalized());
        Assert.Equal(normalized2, inPlace2.NormalizeInPlace(out _));
        Assert.Equal(normalized2.X, normalized2.Y);

        Fixed64 component3d = Fixed64.FromRaw(154_981_286L);
        var vector3 = new Vector3d(component3d, component3d, component3d);
        Assert.True(RequiresWideRepair(vector3));
        Vector3d normalized3 = vector3.Normalized;
        Vector3d inPlace3 = vector3;
        Assert.True(normalized3.IsNormalized());
        Assert.Equal(normalized3, inPlace3.NormalizeInPlace(out _));
        Assert.Equal(normalized3.X, normalized3.Y);
        Assert.Equal(normalized3.X, normalized3.Z);

        Fixed64 component4d = Fixed64.FromRaw(134_217_736L);
        var vector4 = new Vector4d(component4d, component4d, component4d, component4d);
        Assert.True(RequiresWideRepair(vector4));
        Vector4d normalized4 = vector4.Normalized;
        Vector4d inPlace4 = vector4;
        Assert.True(normalized4.IsNormalized());
        Assert.Equal(normalized4, inPlace4.NormalizeInPlace(out _));
        Assert.Equal(normalized4.X, normalized4.Y);
        Assert.Equal(normalized4.X, normalized4.Z);
        Assert.Equal(normalized4.X, normalized4.W);

        var quaternion = new FixedQuaternion(component4d, component4d, component4d, component4d);
        FixedQuaternion normalizedQuaternion = quaternion.Normalized;
        FixedQuaternion inPlaceQuaternion = quaternion;
        Assert.True(normalizedQuaternion.IsNormalized());
        Assert.Equal(normalizedQuaternion, inPlaceQuaternion.NormalizeInPlace());
        Assert.Equal(normalizedQuaternion.X, normalizedQuaternion.Y);
        Assert.Equal(normalizedQuaternion.X, normalizedQuaternion.Z);
        Assert.Equal(normalizedQuaternion.X, normalizedQuaternion.W);
    }

    private static bool RequiresWideRepair(Vector2d source)
    {
        Fixed64 squared = source.X * source.X + source.Y * source.Y;
        if (squared <= FixedMath.ScaleSafeMagnitudeSquaredThreshold)
            return false;
        Fixed64 magnitude = FixedMath.Sqrt(squared);
        var provisional = new Vector2d(
            FixedMath.FastDiv(source.X, magnitude),
            FixedMath.FastDiv(source.Y, magnitude));
        return !provisional.IsNormalized();
    }

    private static bool RequiresWideRepair(Vector3d source)
    {
        Fixed64 squared = source.X * source.X + source.Y * source.Y + source.Z * source.Z;
        if (squared <= FixedMath.ScaleSafeMagnitudeSquaredThreshold)
            return false;
        Fixed64 magnitude = FixedMath.Sqrt(squared);
        var provisional = new Vector3d(
            FixedMath.FastDiv(source.X, magnitude),
            FixedMath.FastDiv(source.Y, magnitude),
            FixedMath.FastDiv(source.Z, magnitude));
        return !provisional.IsNormalized();
    }

    private static bool RequiresWideRepair(Vector4d source)
    {
        Fixed64 squared = source.X * source.X + source.Y * source.Y + source.Z * source.Z + source.W * source.W;
        if (squared <= FixedMath.ScaleSafeMagnitudeSquaredThreshold)
            return false;
        Fixed64 magnitude = FixedMath.Sqrt(squared);
        var provisional = new Vector4d(
            FixedMath.FastDiv(source.X, magnitude),
            FixedMath.FastDiv(source.Y, magnitude),
            FixedMath.FastDiv(source.Z, magnitude),
            FixedMath.FastDiv(source.W, magnitude));
        return !provisional.IsNormalized();
    }

    [Fact]
    public void Vector2d_NormalizationSurfaces_RepairRoundedMagnitudeFalsePositives()
    {
        Vector2d[] sources =
        {
            new(Fixed64.FromRaw(NearNegativeUnitRaw), Fixed64.FromRaw(MinorComponentRaw)),
            new(Fixed64.FromRaw(NearPositiveUnitRaw), Fixed64.FromRaw(-MinorComponentRaw)),
            new(Fixed64.FromRaw(MinorComponentRaw), Fixed64.FromRaw(NearNegativeUnitRaw))
        };

        foreach (Vector2d source in sources)
        {
            Assert.False(source.IsNormalized());
            Vector2d normalized = source.Normalized;
            Vector2d inPlace = source;

            Assert.True(normalized.IsNormalized());
            Assert.Equal(normalized, inPlace.NormalizeInPlace(out Fixed64 magnitude));
            Assert.Equal(source.Magnitude, magnitude);
        }
    }

    [Fact]
    public void Vector3d_NormalizationSurfaces_RepairRoundedMagnitudeFalsePositives()
    {
        Vector3d[] sources =
        {
            new(Fixed64.FromRaw(NearNegativeUnitRaw), Fixed64.FromRaw(MinorComponentRaw), Fixed64.Zero),
            new(Fixed64.Zero, Fixed64.FromRaw(NearPositiveUnitRaw), Fixed64.FromRaw(-MinorComponentRaw)),
            new(Fixed64.FromRaw(MinorComponentRaw), Fixed64.Zero, Fixed64.FromRaw(NearNegativeUnitRaw))
        };

        foreach (Vector3d source in sources)
        {
            Assert.False(source.IsNormalized());
            Vector3d normalized = source.Normalized;
            Vector3d inPlace = source;

            Assert.True(normalized.IsNormalized());
            Assert.Equal(normalized, inPlace.NormalizeInPlace(out Fixed64 magnitude));
            Assert.Equal(source.Magnitude, magnitude);
        }
    }

    [Fact]
    public void Vector4d_NormalizationSurfaces_RepairRoundedMagnitudeFalsePositives()
    {
        Vector4d[] sources =
        {
            new(Fixed64.FromRaw(NearNegativeUnitRaw), Fixed64.FromRaw(MinorComponentRaw), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.Zero, Fixed64.FromRaw(NearPositiveUnitRaw), Fixed64.FromRaw(-MinorComponentRaw), Fixed64.Zero),
            new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(MinorComponentRaw), Fixed64.FromRaw(NearNegativeUnitRaw))
        };

        foreach (Vector4d source in sources)
        {
            Assert.False(source.IsNormalized());
            Vector4d normalized = source.Normalized;
            Vector4d inPlace = source;

            Assert.True(normalized.IsNormalized());
            Assert.Equal(normalized, inPlace.NormalizeInPlace(out Fixed64 magnitude));
            Assert.Equal(source.Magnitude, magnitude);
        }
    }

    [Fact]
    public void FixedQuaternion_NormalizationSurfaces_UseTheSquaredMagnitudeContract()
    {
        FixedQuaternion[] sources =
        {
            new(Fixed64.FromRaw(NearNegativeUnitRaw), Fixed64.FromRaw(MinorComponentRaw), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.Zero, Fixed64.FromRaw(NearPositiveUnitRaw), Fixed64.FromRaw(-MinorComponentRaw), Fixed64.Zero),
            new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(MinorComponentRaw), Fixed64.FromRaw(NearNegativeUnitRaw))
        };

        foreach (FixedQuaternion source in sources)
        {
            Assert.False(source.IsNormalized());
            FixedQuaternion normalized = source.Normalized;
            FixedQuaternion inPlace = source;

            Assert.True(normalized.IsNormalized());
            Assert.Equal(normalized, inPlace.NormalizeInPlace());
        }
    }

    [Fact]
    public void AlreadyNormalizedValues_ArePreservedAcrossEveryType()
    {
        Fixed64 minor = Fixed64.FromRaw(1_045_500);
        var vector2 = new Vector2d(Fixed64.One, minor);
        var vector3 = new Vector3d(Fixed64.One, minor, Fixed64.Zero);
        var vector4 = new Vector4d(Fixed64.One, minor, Fixed64.Zero, Fixed64.Zero);
        var quaternion = new FixedQuaternion(Fixed64.One, minor, Fixed64.Zero, Fixed64.Zero);

        Assert.True(vector2.IsNormalized());
        Assert.True(vector3.IsNormalized());
        Assert.True(vector4.IsNormalized());
        Assert.True(quaternion.IsNormalized());
        Assert.Equal(vector2, vector2.Normalized);
        Assert.Equal(vector3, vector3.Normalized);
        Assert.Equal(vector4, vector4.Normalized);
        Assert.Equal(quaternion, quaternion.Normalized);
        Assert.False(default(FixedQuaternion).IsNormalized());
    }

    [Fact]
    public void GetDirection_WithTinyRepresentableDifference_ReturnsAcceptedUnitDirections()
    {
        Vector2d direction2d = Vector2d.GetDirection(
            Vector2d.Zero,
            new Vector2d(Fixed64.FromRaw(1), Fixed64.FromRaw(2)));
        Vector3d direction3d = Vector3d.GetDirection(
            Vector3d.Zero,
            new Vector3d(Fixed64.FromRaw(1), Fixed64.FromRaw(2), Fixed64.FromRaw(3)));

        Assert.True(direction2d.IsNormalized());
        Assert.True(direction3d.IsNormalized());
        Assert.True(FixedMath.Abs(direction2d.Y - direction2d.X * Fixed64.Two) <= Fixed64.Epsilon);
        Assert.True(FixedMath.Abs(direction3d.Y - direction3d.X * Fixed64.Two) <= Fixed64.Epsilon);
        Assert.True(FixedMath.Abs(direction3d.Z - direction3d.X * Fixed64.Three) <= Fixed64.Epsilon);
    }
}
