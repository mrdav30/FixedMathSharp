using System;
using Xunit;

namespace FixedMathSharp.Tests;

public class CoordinateConvention3dTests
{
    [Fact]
    public void PositiveZForward_ExposesCanonicalBasis()
    {
        CoordinateConvention3d convention = CoordinateConvention3d.PositiveZForward;

        Assert.Equal(CoordinateConvention3d.Canonical, convention);
        Assert.Equal(Axis3d.PositiveX, convention.RightAxis);
        Assert.Equal(Axis3d.PositiveY, convention.UpAxis);
        Assert.Equal(Axis3d.PositiveZ, convention.ForwardAxis);
        Assert.Equal(Vector3d.Right, convention.Right);
        Assert.Equal(Vector3d.Left, convention.Left);
        Assert.Equal(Vector3d.Up, convention.Up);
        Assert.Equal(Vector3d.Down, convention.Down);
        Assert.Equal(Vector3d.Forward, convention.Forward);
        Assert.Equal(Vector3d.Backward, convention.Backward);
    }

    [Fact]
    public void NegativeZForward_ExposesZFlippedForwardAndBackward()
    {
        CoordinateConvention3d convention = CoordinateConvention3d.NegativeZForward;

        Assert.Equal(Axis3d.PositiveX, convention.RightAxis);
        Assert.Equal(Axis3d.PositiveY, convention.UpAxis);
        Assert.Equal(Axis3d.NegativeZ, convention.ForwardAxis);
        Assert.Equal(Vector3d.Right, convention.Right);
        Assert.Equal(Vector3d.Left, convention.Left);
        Assert.Equal(Vector3d.Up, convention.Up);
        Assert.Equal(Vector3d.Down, convention.Down);
        Assert.Equal(Vector3d.Backward, convention.Forward);
        Assert.Equal(Vector3d.Forward, convention.Backward);
    }

    [Fact]
    public void XForwardZUp_ExposesUnrealStyleBasisWithoutEngineSpecificApiNames()
    {
        CoordinateConvention3d convention = CoordinateConvention3d.XForwardZUp;

        Assert.Equal(Axis3d.PositiveY, convention.RightAxis);
        Assert.Equal(Axis3d.PositiveZ, convention.UpAxis);
        Assert.Equal(Axis3d.PositiveX, convention.ForwardAxis);
        Assert.Equal(Vector3d.Up, convention.Right);
        Assert.Equal(Vector3d.Down, convention.Left);
        Assert.Equal(Vector3d.Forward, convention.Up);
        Assert.Equal(Vector3d.Backward, convention.Down);
        Assert.Equal(Vector3d.Right, convention.Forward);
        Assert.Equal(Vector3d.Left, convention.Backward);
    }

    [Fact]
    public void CustomConvention_ExposesNegativeSignedAxes()
    {
        var convention = new CoordinateConvention3d(Axis3d.NegativeX, Axis3d.NegativeY, Axis3d.NegativeZ);

        Assert.Equal(Vector3d.Left, convention.Right);
        Assert.Equal(Vector3d.Right, convention.Left);
        Assert.Equal(Vector3d.Down, convention.Up);
        Assert.Equal(Vector3d.Up, convention.Down);
        Assert.Equal(Vector3d.Backward, convention.Forward);
        Assert.Equal(Vector3d.Forward, convention.Backward);
    }

    [Fact]
    public void PositiveZForward_DirectionConversionIsIdentity()
    {
        CoordinateConvention3d convention = CoordinateConvention3d.PositiveZForward;
        var direction = new Vector3d(2, -3, 4);

        Assert.Equal(direction, convention.ToCanonicalDirection(direction));
        Assert.Equal(direction, convention.FromCanonicalDirection(direction));
    }

    [Fact]
    public void NegativeZForward_DirectionConversionFlipsZ()
    {
        CoordinateConvention3d convention = CoordinateConvention3d.NegativeZForward;
        var externalDirection = new Vector3d(2, -3, -4);
        var canonicalDirection = new Vector3d(2, -3, 4);

        Assert.Equal(canonicalDirection, convention.ToCanonicalDirection(externalDirection));
        Assert.Equal(externalDirection, convention.FromCanonicalDirection(canonicalDirection));
    }

    [Fact]
    public void XForwardZUp_DirectionConversionPermutesAxes()
    {
        CoordinateConvention3d convention = CoordinateConvention3d.XForwardZUp;
        var externalDirection = new Vector3d(11, 22, 33);
        var canonicalDirection = new Vector3d(22, 33, 11);

        Assert.Equal(Vector3d.Forward, convention.ToCanonicalDirection(Vector3d.Right));
        Assert.Equal(Vector3d.Right, convention.ToCanonicalDirection(Vector3d.Up));
        Assert.Equal(Vector3d.Up, convention.ToCanonicalDirection(Vector3d.Forward));
        Assert.Equal(canonicalDirection, convention.ToCanonicalDirection(externalDirection));
        Assert.Equal(externalDirection, convention.FromCanonicalDirection(canonicalDirection));
    }

    [Fact]
    public void CustomConvention_DirectionConversionAppliesSignedAxes()
    {
        var convention = new CoordinateConvention3d(Axis3d.NegativeX, Axis3d.NegativeY, Axis3d.NegativeZ);
        var externalDirection = new Vector3d(2, -3, 4);
        var canonicalDirection = new Vector3d(-2, 3, -4);

        Assert.Equal(canonicalDirection, convention.ToCanonicalDirection(externalDirection));
        Assert.Equal(externalDirection, convention.FromCanonicalDirection(canonicalDirection));
    }

    [Fact]
    public void DirectionConversion_RoundTripsPresetDirections()
    {
        CoordinateConvention3d[] conventions =
        {
            CoordinateConvention3d.PositiveZForward,
            CoordinateConvention3d.NegativeZForward,
            CoordinateConvention3d.XForwardZUp,
        };

        Vector3d[] canonicalDirections =
        {
            Vector3d.Right,
            Vector3d.Left,
            Vector3d.Up,
            Vector3d.Down,
            Vector3d.Forward,
            Vector3d.Backward,
        };

        foreach (CoordinateConvention3d convention in conventions)
        {
            foreach (Vector3d canonicalDirection in canonicalDirections)
            {
                Vector3d conventionDirection = convention.FromCanonicalDirection(canonicalDirection);

                Assert.Equal(canonicalDirection, convention.ToCanonicalDirection(conventionDirection));
            }
        }
    }

    [Theory]
    [InlineData((Axis3d)(-1), Axis3d.PositiveY, Axis3d.PositiveZ, "rightAxis")]
    [InlineData((Axis3d)42, Axis3d.PositiveY, Axis3d.PositiveZ, "rightAxis")]
    [InlineData(Axis3d.PositiveX, (Axis3d)42, Axis3d.PositiveZ, "upAxis")]
    [InlineData(Axis3d.PositiveX, Axis3d.PositiveY, (Axis3d)42, "forwardAxis")]
    public void Constructor_InvalidAxis_Throws(Axis3d rightAxis, Axis3d upAxis, Axis3d forwardAxis, string expectedParamName)
    {
        var exception = Assert.Throws<ArgumentOutOfRangeException>(() => new CoordinateConvention3d(rightAxis, upAxis, forwardAxis));

        Assert.Equal(expectedParamName, exception.ParamName);
    }

    [Theory]
    [InlineData(Axis3d.PositiveX, Axis3d.NegativeX, Axis3d.PositiveZ, "upAxis")]
    [InlineData(Axis3d.PositiveX, Axis3d.PositiveY, Axis3d.NegativeX, "forwardAxis")]
    [InlineData(Axis3d.PositiveX, Axis3d.PositiveY, Axis3d.NegativeY, "forwardAxis")]
    public void Constructor_DuplicateAbsoluteAxis_Throws(Axis3d rightAxis, Axis3d upAxis, Axis3d forwardAxis, string expectedParamName)
    {
        var exception = Assert.Throws<ArgumentException>(() => new CoordinateConvention3d(rightAxis, upAxis, forwardAxis));

        Assert.Equal(expectedParamName, exception.ParamName);
    }

    [Fact]
    public void Equality_UsesAllAxes()
    {
        CoordinateConvention3d canonical = CoordinateConvention3d.Canonical;
        CoordinateConvention3d sameCanonical = new(Axis3d.PositiveX, Axis3d.PositiveY, Axis3d.PositiveZ);
        CoordinateConvention3d differentRight = new(Axis3d.NegativeX, Axis3d.PositiveY, Axis3d.PositiveZ);
        CoordinateConvention3d differentUp = new(Axis3d.PositiveX, Axis3d.NegativeY, Axis3d.PositiveZ);
        CoordinateConvention3d differentForward = CoordinateConvention3d.NegativeZForward;

        Assert.Equal(canonical, sameCanonical);
        Assert.True(canonical == sameCanonical);
        Assert.False(canonical != sameCanonical);
        Assert.NotEqual(canonical, differentRight);
        Assert.NotEqual(canonical, differentUp);
        Assert.NotEqual(canonical, differentForward);
        Assert.True(canonical != differentForward);
        Assert.False(canonical == differentForward);
        Assert.True(canonical.Equals((object)sameCanonical));
        Assert.False(canonical.Equals("not-a-convention"));
        Assert.False(canonical.Equals(null));
        Assert.Equal(canonical.GetHashCode(), sameCanonical.GetHashCode());
    }
}
