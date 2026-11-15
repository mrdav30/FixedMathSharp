//=======================================================================
// Fixed3x3Assertions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

/// <summary>Provides FluentAssertions checks for a 3×3 fixed-point matrix.</summary>
/// <remarks>
/// Approximate checks use Q32.32 arithmetic, including its rounding and saturation.
/// Failures use the active FluentAssertions test framework and assertion scope.
/// </remarks>
public sealed class Fixed3x3Assertions : FixedStructAssertions<Fixed3x3, Fixed3x3Assertions>
{
    /// <summary>Creates an assertion object for a 3×3 fixed-point matrix.</summary>
    /// <param name="value">The subject to capture by value.</param>
    public Fixed3x3Assertions(Fixed3x3 value) : base(value) { }

    /// <summary>Gets the subject label used in assertion failure messages.</summary>
    /// <value><c>fixed3x3 matrix</c>.</value>
    protected override string Identifier => "fixed3x3 matrix";

    /// <summary>Asserts that every matrix entry is within tolerance of its corresponding expected entry.</summary>
    /// <param name="expected">The matrix of expected entries.</param>
    /// <param name="tolerance">The inclusive absolute error allowed per matrix entry. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed3x3Assertions> BeApproximately(
        Fixed3x3 expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreComponentApproximatelyEqual(Subject, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed3x3 matrix to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, Subject);

        return new AndConstraint<Fixed3x3Assertions>(this);
    }

    /// <summary>Asserts that at least one matrix entry differs from its corresponding entry by more than the tolerance.</summary>
    /// <param name="expected">The matrix to compare against.</param>
    /// <param name="tolerance">The inclusive absolute error allowed per matrix entry. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed3x3Assertions> NotBeApproximately(
        Fixed3x3 expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);

        CurrentAssertionChain
            .ForCondition(!FixedAssertionHelpers.AreComponentApproximatelyEqual(Subject, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed3x3 matrix not to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, Subject);

        return new AndConstraint<Fixed3x3Assertions>(this);
    }

    /// <summary>Asserts exact equality with <see cref="Fixed3x3.Identity"/>.</summary>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed3x3Assertions> BeIdentity(
        string because = "",
        params object[] becauseArgs)
    {
        return Be(Fixed3x3.Identity, because, becauseArgs);
    }

    /// <summary>Asserts that the basis-row lengths are within tolerance of the expected scale magnitudes.</summary>
    /// <param name="expected">The expected unsigned scale factors for X, Y, and Z.</param>
    /// <param name="tolerance">The inclusive absolute error allowed per axis scale factor. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed3x3Assertions> HaveScaleApproximately(
        Vector3d expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector3d scale = Fixed3x3.ExtractScaleMagnitudes(Subject);

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreComponentApproximatelyEqual(scale, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed3x3 matrix scale magnitudes to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, scale);

        return new AndConstraint<Fixed3x3Assertions>(this);
    }

    /// <summary>Asserts that each matrix row has approximately unit length.</summary>
    /// <param name="tolerance">The inclusive absolute deviation from unit length. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <remarks>This checks basis lengths only. The rows may be nonorthogonal.</remarks>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed3x3Assertions> HaveNormalizedAxes(
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.HasUnitAxes(Subject, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed3x3 matrix axes to be normalized within +/- {0}{reason}, but found {1}.", limit, Subject);

        return new AndConstraint<Fixed3x3Assertions>(this);
    }
}
