using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairGeometryTests
{
    [Fact]
    public void Prepare_PreservesCoprimeRotationsWithoutIntroducingACommonScale()
    {
        Fixed64 rawOne = Fixed64.FromRaw(1);
        var firstRotation = new FixedQuaternion(rawOne, Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        var secondRotation = new FixedQuaternion(Fixed64.Zero, rawOne, Fixed64.Zero, Fixed64.FromRaw((1L << 32) + 2));
        var firstAxis = new Vector3d(Fixed64.One, rawOne, Fixed64.Zero);
        var secondAxis = new Vector3d(Fixed64.Zero, Fixed64.One, rawOne);
        Assert.True(firstRotation.IsNormalized());
        Assert.True(secondRotation.IsNormalized());
        Assert.True(firstAxis.IsNormalized());
        Assert.True(secondAxis.IsNormalized());
        var geometry = new CylinderPairGeometry(Vector3d.Zero, firstRotation, firstAxis,
            Signed192.Raw(rawOne), rawOne, Vector3d.Zero, secondRotation, secondAxis,
            Signed192.Raw(rawOne), rawOne);

        // Integer quaternion matrices, without a fixed-point intermediate:
        // inverse rotation around X applied to rotation around Y.
        BigInteger q = BigInteger.One << 32;
        BigInteger d1 = q * q + 1;
        BigInteger d2 = (q + 2) * (q + 2) + 1;
        BigInteger z = (q + 2) * (q + 2) - 1;
        Assert.Equal(2 * q * d1 * d2, Integer(geometry.RawScale));
        AssertAxis(geometry.FirstHalf, q * d1 * d2, d1 * d2, 0);
        AssertAxis(geometry.SecondHalf, 2 * (q + 2) * d1,
            (q * q - 1) * q * d2 + 2 * q * z,
            -2 * q * q * d2 + (q * q - 1) * z);
        Assert.Equal(2 * q * d1 * d2, Integer(geometry.FirstRadius));
    }

    [Fact]
    public void Prepare_PreservesCoprimeAuthoredAxisAndHalfRawLength()
    {
        Vector3d axis = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero);
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, axis,
            Signed192.Raw(Fixed64.FromRaw(1)), Fixed64.FromRaw(1), Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Forward, Signed192.Raw(Fixed64.FromRaw(1)), Fixed64.FromRaw(1));
        BigInteger q = BigInteger.One << 32;
        Assert.True(axis.IsNormalized());
        Assert.Equal(2 * q, Integer(geometry.RawScale));
        AssertAxis(geometry.FirstHalf, axis.X.m_rawValue, axis.Y.m_rawValue, 0);
        AssertAxis(geometry.SecondHalf, 0, 0, q);
        Assert.Equal(2 * q, Integer(geometry.FirstRadius));
        AssertAxis(geometry.GetCapOffset(-1, 1), -axis.X.m_rawValue, -axis.Y.m_rawValue, q);
    }

    [Fact]
    public void Prepare_RemovesCommonScaleWithoutRoundingAuthoredCoordinates()
    {
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Signed192.Raw((Fixed64)2), Fixed64.One, new Vector3d((Fixed64)7 / 4, (Fixed64)7 / 4, (Fixed64)5 / 4),
            FixedQuaternion.Identity, Vector3d.Right, Signed192.Raw((Fixed64)2), Fixed64.One);
        Assert.Equal(BigInteger.One, Integer(geometry.RawScale));
        AssertAxis(geometry.FirstAxis, 0, 1, 0);
        AssertAxis(geometry.SecondAxis, 1, 0, 0);
        BigInteger q = BigInteger.One << 32;
        AssertAxis(geometry.GetCapOffset(1, 1), -3 * q / 4, -3 * q / 4, -5 * q / 4);
        AssertAxis(geometry.GetCapOffset(-1, -1), -11 * q / 4, -11 * q / 4, -5 * q / 4);
        Assert.Equal(q, Integer(geometry.FirstRadius));
        Assert.Equal(q, Integer(geometry.SecondRadius));
        Assert.True(geometry.ValueShift < 100);
    }

    [Fact]
    public void Prepare_PreservesExactRelativeQuaternionFrame()
    {
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var geometry = new CylinderPairGeometry(Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s), Vector3d.Right,
            Signed192.Raw((Fixed64)10), Fixed64.One, new Vector3d(0, 1, 8),
            new FixedQuaternion(Fixed64.Zero, s, Fixed64.Zero, 2 * s), Vector3d.Left,
            Signed192.Raw((Fixed64)10), Fixed64.One);
        BigInteger q = BigInteger.One << 32;
        Assert.Equal((BigInteger)5, Integer(geometry.RawScale));
        AssertAxis(geometry.FirstAxis, 1, 0, 0);
        AssertAxis(geometry.SecondAxis, 7, 0, 24);
        AssertAxis(geometry.FirstHalf, 25 * q, 0, 0);
        AssertAxis(geometry.SecondHalf, 7 * q, 0, 24 * q);
        AssertAxis(geometry.GetCapOffset(1, 1), 0, -5 * q, 0);
        Assert.Equal(5 * q, Integer(geometry.FirstRadius));
    }

    [Fact]
    public void Prepare_PreservesOppositeLimitCenterDifferencesAndHalfRawAxes()
    {
        var geometry = new CylinderPairGeometry(new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Forward, Signed192.Raw(Fixed64.FromRaw(1)), Fixed64.FromRaw(1),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.Up,
            Signed192.Raw(Fixed64.FromRaw(1)), Fixed64.FromRaw(1));
        Assert.Equal((BigInteger)2, Integer(geometry.RawScale));
        AssertAxis(geometry.CenterDifference, 2 * (BigInteger)ulong.MaxValue, 0, 0);
        AssertAxis(geometry.FirstHalf, 0, 0, 1);
        AssertAxis(geometry.SecondHalf, 0, 1, 0);
        AssertAxis(geometry.PlaneU, 1, 0, 0);
        AssertAxis(geometry.PlaneW, 0, 1, 0);
    }

    [Fact]
    public void Invariants_MatchExactDotProductsAndClearPaddedSlots()
    {
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Signed192.Raw((Fixed64)2), Fixed64.One, new Vector3d(2, 3, 4), FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw((Fixed64)2), (Fixed64)2);
        const int words = 12;
        ulong[] values = new ulong[11 * words];
        sbyte[] signs = new sbyte[11];
        Array.Fill(values, ulong.MaxValue);
        Array.Fill(signs, (sbyte)-1);
        geometry.WriteInvariants(1, 1, values, signs, words);
        BigInteger q = BigInteger.One << 32;
        BigInteger[] expected = { 1, 1, 1, q * q, 4 * q * q, 21 * q * q, -q, 4 * q, 1, 0, -q };
        for (int index = 0; index < expected.Length; index++)
        {
            BigInteger actual = BigInteger.Zero;
            for (int word = words - 1; word >= 0; word--)
                actual = (actual << 64) | values[index * words + word];
            Assert.Equal(expected[index], actual * signs[index]);
            Assert.Equal(expected[index].Sign, signs[index]);
        }
    }

    private static void AssertAxis(WideAxis3 axis, BigInteger x, BigInteger y, BigInteger z)
    {
        Assert.Equal(x, Integer(axis.X));
        Assert.Equal(y, Integer(axis.Y));
        Assert.Equal(z, Integer(axis.Z));
    }

    private static BigInteger Integer(Signed192 value) =>
        ((BigInteger)(long)value.High << 128) | ((BigInteger)value.Middle << 64) | value.Low;

    private static BigInteger Integer(Signed320 value) =>
        ((BigInteger)(long)value.Word4 << 256) | ((BigInteger)value.Word3 << 192)
        | ((BigInteger)value.Word2 << 128) | ((BigInteger)value.Word1 << 64) | value.Word0;
}
