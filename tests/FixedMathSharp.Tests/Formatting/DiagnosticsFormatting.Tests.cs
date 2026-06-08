using FixedMathSharp.Bounds;
using System;
using System.Globalization;
using Xunit;

namespace FixedMathSharp.Tests;

public class DiagnosticsFormattingTests
{
    [Fact]
    public void Fixed64_FormattableAndTryFormat_UseInvariantDiagnosticsShape()
    {
        Fixed64 value = Fixed64.FromDouble(1234.5);

        Assert.IsAssignableFrom<IFormattable>(value);
        Assert.IsAssignableFrom<ISpanFormattable>(value);
        Assert.Equal("1,234.50", ((IFormattable)value).ToString("N2", CultureInfo.InvariantCulture));

        Span<char> destination = stackalloc char[32];

        Assert.True(value.TryFormat(destination, out int charsWritten, "0.00", CultureInfo.InvariantCulture));
        Assert.Equal("1234.50", new string(destination[..charsWritten]));

        Span<char> tooSmall = stackalloc char[4];

        Assert.False(value.TryFormat(tooSmall, out charsWritten, "0.00", CultureInfo.InvariantCulture));
        Assert.Equal(0, charsWritten);
    }

    [Fact]
    public void VectorAndQuaternionFormatting_UseSharedComponentFormatShape()
    {
        var vector2 = new Vector2d(Fixed64.FromDouble(1.25), Fixed64.FromDouble(-2.5));
        var vector3 = new Vector3d(Fixed64.FromDouble(1.25), Fixed64.FromDouble(-2.5), Fixed64.FromDouble(3.75));
        var vector4 = new Vector4d(Fixed64.FromDouble(1.25), Fixed64.FromDouble(-2.5), Fixed64.FromDouble(3.75), Fixed64.FromDouble(-4.5));
        var quaternion = new FixedQuaternion(Fixed64.FromDouble(1.25), Fixed64.FromDouble(-2.5), Fixed64.FromDouble(3.75), Fixed64.FromDouble(-4.5));

        AssertFormatted(vector2, "(1.25, -2.50)");
        AssertFormatted(vector3, "(1.25, -2.50, 3.75)");
        AssertFormatted(vector4, "(1.25, -2.50, 3.75, -4.50)");
        AssertFormatted(quaternion, "(1.25, -2.50, 3.75, -4.50)");
    }

    [Fact]
    public void MatrixRangeAndSphereFormatting_UseSharedDiagnosticsFormatShape()
    {
        var matrix3x3 = new Fixed3x3(
            Fixed64.FromDouble(1.25), Fixed64.FromDouble(-2.5), Fixed64.FromDouble(3.75),
            Fixed64.FromDouble(4.5), Fixed64.FromDouble(-5.25), Fixed64.FromDouble(6.75),
            Fixed64.FromDouble(-7.5), Fixed64.FromDouble(8.25), Fixed64.FromDouble(-9.75));
        var matrix4x4 = new Fixed4x4(
            Fixed64.FromDouble(1.25), Fixed64.FromDouble(-2.5), Fixed64.FromDouble(3.75), Fixed64.FromDouble(-4.5),
            Fixed64.FromDouble(5.25), Fixed64.FromDouble(-6.75), Fixed64.FromDouble(7.5), Fixed64.FromDouble(-8.25),
            Fixed64.FromDouble(9.75), Fixed64.FromDouble(-10.5), Fixed64.FromDouble(11.25), Fixed64.FromDouble(-12.75),
            Fixed64.FromDouble(13.5), Fixed64.FromDouble(-14.25), Fixed64.FromDouble(15.75), Fixed64.FromDouble(-16.5));
        var range = new FixedRange(Fixed64.FromDouble(-1.25), Fixed64.FromDouble(2.5));
        var sphere = new FixedBoundSphere(
            new Vector3d(Fixed64.FromDouble(1.25), Fixed64.FromDouble(-2.5), Fixed64.FromDouble(3.75)),
            Fixed64.FromDouble(4.5));

        AssertFormatted(matrix3x3, "[1.25, -2.50, 3.75; 4.50, -5.25, 6.75; -7.50, 8.25, -9.75]");
        AssertFormatted(matrix4x4, "[1.25, -2.50, 3.75, -4.50; 5.25, -6.75, 7.50, -8.25; 9.75, -10.50, 11.25, -12.75; 13.50, -14.25, 15.75, -16.50]");
        AssertFormatted(range, "-1.25 - 2.50");
        AssertFormatted(sphere, "{Center:(1.25, -2.50, 3.75) Radius:4.50}");
    }

    [Fact]
    public void FixedDiagnosticsFormatter_GrowsBeyondInitialAndFirstRentedBuffer()
    {
        string result = FixedDiagnosticsFormatter.ToString((Span<char> destination, out int charsWritten) =>
        {
            const int RequiredLength = 700;
            if (destination.Length < RequiredLength)
            {
                charsWritten = 0;
                return false;
            }

            destination[..RequiredLength].Fill('x');
            charsWritten = RequiredLength;
            return true;
        });

        Assert.Equal(new string('x', 700), result);

        Span<char> empty = Span<char>.Empty;
        int written = 0;

        Assert.False(FixedDiagnosticsFormatter.Append('x', empty, ref written));
        Assert.Equal(0, written);
    }

    private static void AssertFormatted<T>(T value, string expected)
        where T : IFormattable, ISpanFormattable
    {
        Assert.Equal(expected, value.ToString("0.00", CultureInfo.InvariantCulture));

        Span<char> destination = stackalloc char[256];

        Assert.True(value.TryFormat(destination, out int charsWritten, "0.00", CultureInfo.InvariantCulture));
        Assert.Equal(expected, new string(destination[..charsWritten]));

        Span<char> tooSmall = stackalloc char[4];

        Assert.False(value.TryFormat(tooSmall, out charsWritten, "0.00", CultureInfo.InvariantCulture));
        Assert.Equal(0, charsWritten);
    }
}
