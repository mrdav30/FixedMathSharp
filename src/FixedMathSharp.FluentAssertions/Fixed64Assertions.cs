//=======================================================================
// Fixed64Assertions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

/// <summary>Provides FluentAssertions checks for a fixed-point scalar.</summary>
/// <remarks>
/// Approximate checks use Q32.32 arithmetic, including its rounding and saturation.
/// Failures use the active FluentAssertions test framework and assertion scope.
/// </remarks>
public sealed class Fixed64Assertions : ComparableTypeAssertions<Fixed64, Fixed64Assertions>
{
    /// <summary>Creates an assertion object for a fixed-point scalar.</summary>
    /// <param name="value">The subject to capture by value.</param>
    public Fixed64Assertions(Fixed64 value) : base(value, AssertionChain.GetOrCreate()) { }

    /// <summary>Gets the subject label used in assertion failure messages.</summary>
    /// <value><c>fixed64</c>.</value>
    protected override string Identifier => "fixed64";

    /// <summary>Asserts that the absolute difference from the expected value is within tolerance.</summary>
    /// <param name="expected">The expected fixed-point value.</param>
    /// <param name="tolerance">The inclusive absolute error in fixed-point units. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed64Assertions> BeApproximately(
        Fixed64 expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Fixed64 actual = (Fixed64)Subject;

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreApproximatelyEqual(actual, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed64 to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, actual);

        return new AndConstraint<Fixed64Assertions>(this);
    }

    /// <summary>Asserts that the absolute difference from the supplied value exceeds the tolerance.</summary>
    /// <param name="expected">The fixed-point value to compare against.</param>
    /// <param name="tolerance">The inclusive absolute error in fixed-point units. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed64Assertions> NotBeApproximately(
        Fixed64 expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Fixed64 actual = (Fixed64)Subject;

        CurrentAssertionChain
            .ForCondition((actual - expected).Abs() > limit)
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed64 not to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, actual);

        return new AndConstraint<Fixed64Assertions>(this);
    }

    /// <summary>Asserts that the subject has the given raw Q32.32 payload.</summary>
    /// <param name="expected">The expected signed 64-bit payload, with 32 fractional bits.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed64Assertions> HaveRawValue(
        long expected,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 actual = (Fixed64)Subject;

        CurrentAssertionChain
            .ForCondition(actual.m_rawValue == expected)
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed64 to have raw value {0}{reason}, but found {1}.", expected, actual.m_rawValue);

        return new AndConstraint<Fixed64Assertions>(this);
    }
}
