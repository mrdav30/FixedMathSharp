//=======================================================================
// VectorMagnitudeCeiling.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Xunit;

namespace FixedMathSharp.Tests;

public sealed class VectorMagnitudeCeilingTests
{
    [Fact]
    public void MagnitudeCeiling_RoundsIrrationalLowRawLengthsOutward()
    {
        var planar = new Vector2d(
            Fixed64.MinIncrement,
            Fixed64.MinIncrement);
        var spatial = new Vector3d(
            Fixed64.MinIncrement,
            Fixed64.MinIncrement,
            Fixed64.MinIncrement);

        Assert.True(planar.TryGetMagnitudeCeiling(out Fixed64 planarCeiling));
        Assert.True(spatial.TryGetMagnitudeCeiling(out Fixed64 spatialCeiling));
        Assert.Equal(Fixed64.FromRaw(2L), planarCeiling);
        Assert.Equal(Fixed64.FromRaw(2L), spatialCeiling);
    }

    [Fact]
    public void MagnitudeCeiling_PreservesExactRootsAndRejectsOnlyFinalOverflow()
    {
        Assert.True(
            new Vector2d(3, 4).TryGetMagnitudeCeiling(
                out Fixed64 exact));
        Assert.Equal((Fixed64)5, exact);
        Assert.True(
            Vector3d.Zero.TryGetMagnitudeCeiling(
                out Fixed64 zero));
        Assert.Equal(Fixed64.Zero, zero);

        Assert.False(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.Zero)
            .TryGetMagnitudeCeiling(out Fixed64 rejected));
        Assert.Equal(default, rejected);
    }
}
