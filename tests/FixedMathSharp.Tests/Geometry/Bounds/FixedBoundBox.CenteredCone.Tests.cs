//=======================================================================
// FixedBoundBox.CenteredCone.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public class FixedBoundBoxCenteredConeTests
{
    [Fact]
    public void CenteredFiniteConeBounds_AreTightWithoutMaterializingEndpoints()
    {
        Assert.Equal(
            FixedBoundBox.FromMinMax(
                new Vector3d(-2, -2, -2),
                new Vector3d(2, 2, 2)),
            FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
                Vector3d.Zero,
                Vector3d.Up,
                (Fixed64)4,
                (Fixed64)2));

        Fixed64 rawOne = Fixed64.FromRaw(1L);
        Fixed64 rawTwo = Fixed64.FromRaw(2L);
        FixedBoundBox odd = FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.FromRaw(3L),
            Fixed64.Zero);
        Assert.Equal(new Vector3d(-rawTwo, Fixed64.Zero, Fixed64.Zero), odd.Min);
        Assert.Equal(new Vector3d(rawTwo, Fixed64.Zero, Fixed64.Zero), odd.Max);
        Assert.NotEqual(rawOne, odd.Max.X);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CenteredFiniteConeBounds_ClipOnlyFinalMirroredScalarFace(bool maximumFace)
    {
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector3d axis = maximumFace ? Vector3d.Right : Vector3d.Left;
        FixedBoundBox bounds = FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
            new Vector3d(face, Fixed64.Zero, Fixed64.Zero),
            axis,
            Fixed64.Two,
            Fixed64.Zero);

        Assert.Equal(
            maximumFace ? Fixed64.MaxValue - Fixed64.One : Fixed64.MinValue + Fixed64.One,
            bounds.Min.X == face ? bounds.Max.X : bounds.Min.X);
        Assert.Equal(face, maximumFace ? bounds.Max.X : bounds.Min.X);
    }

    [Fact]
    public void CenteredFiniteConeBounds_RejectInvalidGeometry()
    {
        Assert.Equal(
            "axisDirection",
            Assert.Throws<ArgumentException>(() =>
                FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
                    Vector3d.Zero,
                    Vector3d.Zero,
                    Fixed64.One,
                    Fixed64.One)).ParamName);
        Assert.Equal(
            "height",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
                    Vector3d.Zero,
                    Vector3d.Up,
                    Fixed64.Zero,
                    Fixed64.One)).ParamName);
        Assert.Equal(
            "radius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedBoundBox.FromCenteredFiniteConeClippedToDomain(
                    Vector3d.Zero,
                    Vector3d.Up,
                    Fixed64.One,
                    -Fixed64.One)).ParamName);
    }
}
