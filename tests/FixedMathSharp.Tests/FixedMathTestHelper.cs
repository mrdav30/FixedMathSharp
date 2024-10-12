using Xunit;

namespace FixedMathSharp.Tests
{
    internal static class FixedMathTestHelper
    {
        private static readonly Fixed64 RelativeTolerance = new Fixed64(0.0001); // 0.01%

        /// <summary>
        /// Asserts that the difference between the expected and actual values is within the specified relative tolerance.
        /// </summary>
        /// <param name="expected">The expected value.</param>
        /// <param name="actual">The actual value.</param>
        /// <param name="tolerance">The relative tolerance to apply.</param>
        /// <param name="message">Optional message for assertion failures.</param>
        public static void AssertWithinRelativeTolerance(Fixed64 expected, Fixed64 actual, Fixed64 tolerance = default, string message = "")
        {
            if (tolerance == default)
                tolerance = RelativeTolerance;

            var difference = (actual - expected).Abs();
            var allowedError = expected.Abs() * tolerance;

            Assert.True(difference <= allowedError,
                string.IsNullOrWhiteSpace(message)
                    ? $"Relative error {difference} exceeds tolerance of {allowedError} for expected value {expected}."
                    : message);
        }

        /// <summary>
        /// Asserts that a given <see cref="Fixed64"/> value falls within a specified range [min, max].
        /// If the value is outside the range, the test fails with the provided error message.
        /// </summary>
        /// <param name="value">The <see cref="Fixed64"/> value to check.</param>
        /// <param name="min">The minimum bound of the range.</param>
        /// <param name="max">The maximum bound of the range.</param>
        /// <param name="message">
        /// An optional message to display if the assertion fails.
        /// </param>
        public static void AssertWithinRange(Fixed64 value, Fixed64 min, Fixed64 max, string message = "")
        {
            Assert.True(value >= min && value <= max,
                string.IsNullOrWhiteSpace(message)
                    ? $"Value {value} is not within the range [{min}, {max}]."
                    : message);
        }
    }

}
