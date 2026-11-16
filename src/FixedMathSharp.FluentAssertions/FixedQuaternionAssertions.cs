//=======================================================================
// FixedQuaternionAssertions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;

namespace FixedMathSharp.Assertions;

/// <summary>Provides FluentAssertions checks for a fixed-point quaternion.</summary>
/// <remarks>
/// Approximate checks use Q32.32 arithmetic, including its rounding and saturation.
/// Failures use the active FluentAssertions test framework and assertion scope.
/// </remarks>
public sealed class FixedQuaternionAssertions : FixedStructAssertions<FixedQuaternion, FixedQuaternionAssertions>
{
    /// <summary>Creates an assertion object for a fixed-point quaternion.</summary>
    /// <param name="value">The subject to capture by value.</param>
    public FixedQuaternionAssertions(FixedQuaternion value) : base(value) { }

    /// <summary>Gets the subject label used in assertion failure messages.</summary>
    /// <value><c>fixed quaternion</c>.</value>
    protected override string Identifier => "fixed quaternion";

    /// <summary>Asserts that all four quaternion components are within tolerance of their expected values.</summary>
    /// <param name="expected">The expected quaternion components.</param>
    /// <param name="tolerance">The inclusive absolute error allowed per component. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<FixedQuaternionAssertions> BeApproximately(
        FixedQuaternion expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreComponentApproximatelyEqual(Subject, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed quaternion components to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, Subject);

        return new AndConstraint<FixedQuaternionAssertions>(this);
    }

    /// <summary>Asserts that at least one quaternion component differs by more than the tolerance.</summary>
    /// <param name="expected">The quaternion components to compare against.</param>
    /// <param name="tolerance">The inclusive absolute error allowed per component. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<FixedQuaternionAssertions> NotBeApproximately(
        FixedQuaternion expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);

        CurrentAssertionChain
            .ForCondition(!FixedAssertionHelpers.AreComponentApproximatelyEqual(Subject, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed quaternion not to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, Subject);

        return new AndConstraint<FixedQuaternionAssertions>(this);
    }

    /// <inheritdoc cref="BeApproximately(FixedQuaternion, Fixed64?, string, object[])"/>
    public AndConstraint<FixedQuaternionAssertions> HaveComponentApproximately(
        FixedQuaternion expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        return BeApproximately(expected, tolerance, because, becauseArgs);
    }

    /// <summary>Asserts that two normalized quaternions agree as rotations, allowing opposite signs.</summary>
    /// <param name="expected">The rotation quaternion to compare against. Both quaternions are normalized for the comparison.</param>
    /// <param name="tolerance">The inclusive tolerance on the difference between one and the absolute dot product of the normalized quaternions. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <remarks>Normalization treats the zero quaternion as <see cref="FixedQuaternion.Identity"/>.</remarks>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<FixedQuaternionAssertions> RepresentSameRotationAs(
        FixedQuaternion expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.RepresentsSameRotation(Subject, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed quaternion to represent the same rotation as {0} +/- {1}{reason}, but found {2}.", expected, limit, Subject);

        return new AndConstraint<FixedQuaternionAssertions>(this);
    }

    /// <summary>Asserts exact component equality with <see cref="FixedQuaternion.Identity"/>.</summary>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<FixedQuaternionAssertions> BeIdentity(
        string because = "",
        params object[] becauseArgs)
    {
        return Be(FixedQuaternion.Identity, because, becauseArgs);
    }

    /// <summary>Asserts that the quaternion magnitude is within tolerance of one.</summary>
    /// <param name="tolerance">The inclusive absolute deviation from unit length. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<FixedQuaternionAssertions> BeNormalized(
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Fixed64 magnitude = FixedQuaternion.GetMagnitude(Subject);

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreApproximatelyEqual(magnitude, Fixed64.One, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed quaternion to be normalized within +/- {0}{reason}, but found magnitude {1}.", limit, magnitude);

        return new AndConstraint<FixedQuaternionAssertions>(this);
    }
}
