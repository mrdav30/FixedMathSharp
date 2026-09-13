using Xunit;

namespace FixedMathSharp.Tests;

public sealed class Fixed64BitOperationsTests
{
    [Fact]
    public void CountLeadingZeroes_ShouldHandleEveryHighestBitAndLowerBitPattern()
    {
        for (int leadingZeroes = 0; leadingZeroes < 64; leadingZeroes++)
        {
            ulong highestBit = 1UL << (63 - leadingZeroes);
            Assert.Equal(leadingZeroes, Fixed64.CountLeadingZeroes(highestBit));
            Assert.Equal(leadingZeroes, Fixed64.CountLeadingZeroes(highestBit | (highestBit - 1UL)));
            // Removing the highest bit also reaches the all-zero word at the final position.
            Assert.Equal(leadingZeroes + 1, Fixed64.CountLeadingZeroes(highestBit - 1UL));
        }
    }
}
