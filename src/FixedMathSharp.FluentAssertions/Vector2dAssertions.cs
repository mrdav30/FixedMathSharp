//=======================================================================
// Vector2dAssertions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

/// <summary>Provides FluentAssertions checks for a 2D fixed-point vector.</summary>
/// <remarks>
/// Approximate checks use Q32.32 arithmetic, including its rounding and saturation.
/// Failures use the active FluentAssertions test framework and assertion scope.
/// </remarks>
public sealed class Vector2dAssertions : ComparableTypeAssertions<Vector2d, Vector2dAssertions>
{
    /// <summary>Creates an assertion object for a 2D fixed-point vector.</summary>
    /// <param name="value">The subject to capture by value.</param>
    public Vector2dAssertions(Vector2d value) : base(value, AssertionChain.GetOrCreate()) { }

    /// <summary>Gets the subject label used in assertion failure messages.</summary>
    /// <value><c>vector2d</c>.</value>
    protected override string Identifier => "vector2d";

    /// <summary>Asserts that the magnitude of the fixed-point difference from the expected vector is within tolerance.</summary>
    /// <param name="expected">The expected vector, in the same coordinate space as the subject.</param>
    /// <param name="tolerance">The inclusive distance limit, in the same units as the vector. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Vector2dAssertions> BeApproximately(
        Vector2d expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector2d actual = (Vector2d)Subject;

        CurrentAssertionChain
            .ForCondition((actual - expected).Magnitude <= limit)
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected vector2d to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, actual);

        return new AndConstraint<Vector2dAssertions>(this);
    }

    /// <summary>Asserts that the magnitude of the fixed-point difference from the supplied vector exceeds the tolerance.</summary>
    /// <param name="expected">The vector to compare against, in the same coordinate space as the subject.</param>
    /// <param name="tolerance">The inclusive distance limit, in the same units as the vector. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Vector2dAssertions> NotBeApproximately(
        Vector2d expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector2d actual = (Vector2d)Subject;

        CurrentAssertionChain
            .ForCondition((actual - expected).Magnitude > limit)
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected vector2d not to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, actual);

        return new AndConstraint<Vector2dAssertions>(this);
    }

    /// <summary>Asserts that each vector component is within tolerance of its expected value.</summary>
    /// <param name="expected">The expected components, in the same coordinate space as the subject.</param>
    /// <param name="tolerance">The inclusive absolute error allowed per component. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Vector2dAssertions> HaveComponentApproximately(
        Vector2d expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector2d actual = (Vector2d)Subject;

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreComponentApproximatelyEqual(actual, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected vector2d components to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, actual);

        return new AndConstraint<Vector2dAssertions>(this);
    }

    /// <summary>Asserts that the vector magnitude is within tolerance of the expected length.</summary>
    /// <param name="expected">The expected length, in the same units as the vector components.</param>
    /// <param name="tolerance">The inclusive absolute error in magnitude. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Vector2dAssertions> HaveMagnitudeApproximately(
        Fixed64 expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector2d actual = (Vector2d)Subject;

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreApproximatelyEqual(actual.Magnitude, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected vector2d magnitude to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, actual.Magnitude);

        return new AndConstraint<Vector2dAssertions>(this);
    }

    /// <summary>Asserts that the vector magnitude is within tolerance of one.</summary>
    /// <param name="tolerance">The inclusive absolute deviation from unit length. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Vector2dAssertions> BeNormalized(
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector2d actual = (Vector2d)Subject;

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreApproximatelyEqual(actual.Magnitude, Fixed64.One, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected vector2d to be normalized within +/- {0}{reason}, but found magnitude {1}.", limit, actual.Magnitude);

        return new AndConstraint<Vector2dAssertions>(this);
    }
}
