using Xunit;

namespace FixedMathSharp.Tests;

public sealed class Fixed64BitOperationsTests
{
    [Fact]
    public void CountLeadingZeroes_Zero_ReturnsWordWidth()
    {
        Assert.Equal(64, Fixed64.CountLeadingZeroes(0UL));
    }
}
