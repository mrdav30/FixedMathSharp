//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CircularRimSegmentSupportTests
{
    private const long Scale = 1L << 32;

    [Theory]
    [InlineData(1, 0)]
    [InlineData(-1, 0)]
    [InlineData(1, 1)]
    [InlineData(-1, 1)]
    [InlineData(1, -1)]
    [InlineData(-1, -1)]
    [InlineData(1, 2)]
    [InlineData(-1, 2)]
    [InlineData(1, -2)]
    [InlineData(-1, -2)]
    public void HasSegmentSupport_IncludesEndpointsAndRejectsTheirRawNeighbors(int orientation, int displacement)
    {
        long delta = Coordinate(displacement);
        // At t=3/4, n=(0,20,-15)*orientation, |n_h|=15,
        // and the radial contribution cancels the base offset's axis dot.
        // The resulting projection is -625*delta and support is 625*Scale.
        AssertContract((20, -9, -12), (20 * Scale, -9 * Scale, -12 * Scale),
            (-20 * delta, -100 * Scale * orientation + 9 * delta, 100 * Scale * orientation + 12 * delta),
            25 * Scale, (9 * orientation, 20 * orientation, 0), (-12 * orientation, 0, -20 * orientation),
            Math.Abs(displacement) <= 1);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(-1)]
    [InlineData(2)]
    [InlineData(-2)]
    public void HasSegmentSupport_ZeroRadialAxisDotUsesTheOffsetSign(int displacement)
    {
        AssertContract((1, 0, 0), (Scale, 0, 0), (Coordinate(displacement), 0, 0),
            25 * Scale, (0, 1, 0), (0, 0, 1), Math.Abs(displacement) <= 1);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(-1, 1)]
    [InlineData(1, -1)]
    [InlineData(-1, -1)]
    public void HasSegmentSupport_ZeroEndpointCoefficientRetainsTheUnsquaredRadialSign(int orientation, int endpoint)
    {
        // a=endpoint*H makes one A coefficient zero. The nonzero radial
        // contribution is 12*orientation*Scale, so only its inward sign fits.
        AssertContract((20, -9, -12), (20 * Scale, -9 * Scale, -12 * Scale),
            (20 * Scale * endpoint, -9 * Scale * endpoint, -12 * Scale * endpoint),
            Scale, (9 * orientation, 20 * orientation, 0), (-12 * orientation, 0, -20 * orientation),
            orientation == -endpoint);
    }

    private static void AssertContract((long X, long Y, long Z) axis, (long X, long Y, long Z) halfAxis,
        (long X, long Y, long Z) offset, long radius, (long X, long Y, long Z) first,
        (long X, long Y, long Z) second, bool expected)
    {
        // Independent rational oracle: evaluate n=first+(3/4)*second with
        // integer numerator 4*n. These fixtures have an integer radial norm,
        // so projection feasibility needs neither a radical nor squared signs.
        BigInteger nx = 4 * (BigInteger)first.X + 3 * (BigInteger)second.X;
        BigInteger ny = 4 * (BigInteger)first.Y + 3 * (BigInteger)second.Y;
        BigInteger nz = 4 * (BigInteger)first.Z + 3 * (BigInteger)second.Z;
        BigInteger radialNorm = BigInteger.Abs(nz);
        Assert.Equal(radialNorm * radialNorm, nx * nx + nz * nz);
        Assert.True(radialNorm > 0);
        Assert.Equal(BigInteger.Zero, axis.X * nx + axis.Y * ny + axis.Z * nz);
        BigInteger projection = Dot(axis, offset) * radialNorm + radius * (axis.X * nx + axis.Z * nz);
        BigInteger support = Dot(axis, halfAxis) * radialNorm;
        Assert.True(support > 0);
        Assert.Equal(expected, projection >= -support && projection <= support);

        Span<ulong> coefficients = stackalloc ulong[] { 3, 4 };
        Span<sbyte> coefficientSigns = stackalloc sbyte[] { -1, 1 };
        Span<ulong> cell = stackalloc ulong[8];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, coefficientSigns, 0, cell, out FiniteAxisValueRoot root));
        Span<ulong> data = stackalloc ulong[26 * CylinderContactAlgebra.Words];
        Span<sbyte> signs = stackalloc sbyte[26];
        Span<ulong> scratch = stackalloc ulong[7 * CylinderContactAlgebra.Words];
        CircularRimContactAlgebra.BuildParameter(Wide(offset), Scalar(radius), Signed192.Signed(2 * Scale),
            Wide(first), Wide(second), data, signs);
        bool actual = CircularRimContactAlgebra.HasSegmentSupport(Wide(axis), Wide(halfAxis), Wide(offset), Scalar(radius),
            Wide(first), Wide(second), ref root, data.Slice(5 * CylinderContactAlgebra.Words, 3 * CylinderContactAlgebra.Words), signs.Slice(5, 3), scratch);
        Assert.Equal(expected, actual);
    }

    private static long Coordinate(int displacement) => Math.Sign(displacement) * (Scale + (Math.Abs(displacement) == 2 ? 1 : 0));
    private static BigInteger Dot((long X, long Y, long Z) a, (long X, long Y, long Z) b) =>
        (BigInteger)a.X * b.X + (BigInteger)a.Y * b.Y + (BigInteger)a.Z * b.Z;
    private static WideAxis3 Wide((long X, long Y, long Z) value) => new(Scalar(value.X), Scalar(value.Y), Scalar(value.Z));
    private static Signed320 Scalar(long value) => Signed320.ExtendValue(Signed192.Signed(value));
}
