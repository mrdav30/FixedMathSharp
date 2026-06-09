using System;
using Xunit;

namespace FixedMathSharp.Tests;

public class CoordinateConvention3dTests
{
    [Fact]
    public void PositiveZForward_ExposesCanonicalBasis()
    {
        CoordinateConvention3d convention = CoordinateConvention3d.PositiveZForward;

        Assert.Equal(ForwardAxis.PositiveZ, convention.ForwardAxis);
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

        Assert.Equal(ForwardAxis.NegativeZ, convention.ForwardAxis);
        Assert.Equal(Vector3d.Right, convention.Right);
        Assert.Equal(Vector3d.Left, convention.Left);
        Assert.Equal(Vector3d.Up, convention.Up);
        Assert.Equal(Vector3d.Down, convention.Down);
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
        Assert.Equal(canonicalDirection, convention.ToCanonicalDirection(convention.FromCanonicalDirection(canonicalDirection)));
    }

    [Fact]
    public void Constructor_InvalidForwardAxis_Throws()
    {
        var exception = Assert.Throws<ArgumentOutOfRangeException>(() => new CoordinateConvention3d((ForwardAxis)42));

        Assert.Equal("forwardAxis", exception.ParamName);
    }

    [Fact]
    public void Equality_UsesForwardAxis()
    {
        CoordinateConvention3d positive = CoordinateConvention3d.PositiveZForward;
        CoordinateConvention3d samePositive = new CoordinateConvention3d(ForwardAxis.PositiveZ);
        CoordinateConvention3d negative = CoordinateConvention3d.NegativeZForward;

        Assert.Equal(positive, samePositive);
        Assert.True(positive == samePositive);
        Assert.False(positive != samePositive);
        Assert.NotEqual(positive, negative);
        Assert.True(positive != negative);
        Assert.False(positive == negative);
        Assert.True(positive.Equals((object)samePositive));
        Assert.False(positive.Equals("not-a-convention"));
        Assert.Equal(positive.GetHashCode(), samePositive.GetHashCode());
    }
}
