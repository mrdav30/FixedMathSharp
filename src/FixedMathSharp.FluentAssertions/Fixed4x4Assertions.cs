//=======================================================================
// Fixed4x4Assertions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

/// <summary>Provides FluentAssertions checks for a 4×4 fixed-point matrix.</summary>
/// <remarks>
/// Approximate checks use Q32.32 arithmetic, including its rounding and saturation.
/// Failures use the active FluentAssertions test framework and assertion scope.
/// </remarks>
public sealed class Fixed4x4Assertions : FixedStructAssertions<Fixed4x4, Fixed4x4Assertions>
{
    /// <summary>Creates an assertion object for a 4×4 fixed-point matrix.</summary>
    /// <param name="value">The subject to capture by value.</param>
    public Fixed4x4Assertions(Fixed4x4 value) : base(value) { }

    /// <summary>Gets the subject label used in assertion failure messages.</summary>
    /// <value><c>fixed4x4 matrix</c>.</value>
    protected override string Identifier => "fixed4x4 matrix";

    /// <summary>Asserts that every matrix entry is within tolerance of its corresponding expected entry.</summary>
    /// <param name="expected">The matrix of expected entries.</param>
    /// <param name="tolerance">The inclusive absolute error allowed per matrix entry. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed4x4Assertions> BeApproximately(
        Fixed4x4 expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreComponentApproximatelyEqual(Subject, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed4x4 matrix to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, Subject);

        return new AndConstraint<Fixed4x4Assertions>(this);
    }

    /// <summary>Asserts that at least one matrix entry differs from its corresponding entry by more than the tolerance.</summary>
    /// <param name="expected">The matrix to compare against.</param>
    /// <param name="tolerance">The inclusive absolute error allowed per matrix entry. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed4x4Assertions> NotBeApproximately(
        Fixed4x4 expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);

        CurrentAssertionChain
            .ForCondition(!FixedAssertionHelpers.AreComponentApproximatelyEqual(Subject, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed4x4 matrix not to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, Subject);

        return new AndConstraint<Fixed4x4Assertions>(this);
    }

    /// <summary>Asserts exact equality with <see cref="Fixed4x4.Identity"/>.</summary>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed4x4Assertions> BeIdentity(
        string because = "",
        params object[] becauseArgs)
    {
        return Be(Fixed4x4.Identity, because, becauseArgs);
    }

    /// <summary>Asserts that the last column is exactly (0, 0, 0, 1), as required by <see cref="Fixed4x4.IsAffine"/>.</summary>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed4x4Assertions> BeAffine(
        string because = "",
        params object[] becauseArgs)
    {
        CurrentAssertionChain
            .ForCondition(Subject.IsAffine)
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed4x4 matrix to be affine{reason}, but found {0}.", Subject);

        return new AndConstraint<Fixed4x4Assertions>(this);
    }

    /// <summary>Asserts that the translation row is within tolerance of the expected translation on each axis.</summary>
    /// <param name="expected">The expected X, Y, and Z offsets, in the matrix's translation units.</param>
    /// <param name="tolerance">The inclusive absolute error allowed per component. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed4x4Assertions> HaveTranslationApproximately(
        Vector3d expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector3d translation = Subject.Translation;

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreComponentApproximatelyEqual(translation, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed4x4 matrix translation to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, translation);

        return new AndConstraint<Fixed4x4Assertions>(this);
    }

    /// <summary>Asserts that the basis-row lengths are within tolerance of the expected scale magnitudes.</summary>
    /// <param name="expected">The expected unsigned scale factors for X, Y, and Z.</param>
    /// <param name="tolerance">The inclusive absolute error allowed per axis scale factor. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed4x4Assertions> HaveScaleApproximately(
        Vector3d expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector3d scale = Fixed4x4.ExtractScaleMagnitudes(Subject);

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreComponentApproximatelyEqual(scale, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed4x4 matrix scale magnitudes to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, scale);

        return new AndConstraint<Fixed4x4Assertions>(this);
    }

    /// <summary>Asserts that the quaternion extracted by <see cref="Fixed4x4.Rotation"/> agrees with the expected rotation.</summary>
    /// <param name="expected">The expected quaternion. The extracted and expected quaternions are normalized before comparison, and opposite signs are equivalent.</param>
    /// <param name="tolerance">The inclusive tolerance on the difference between one and the absolute dot product of the normalized quaternions. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <remarks>This assertion examines the extracted rotation. Use <see cref="Fixed4x4.Decompose(Fixed4x4, out Vector3d, out FixedQuaternion, out Vector3d)"/> to check whether the matrix admits a strict scale, rotation, and translation decomposition.</remarks>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed4x4Assertions> HaveRotationApproximately(
        FixedQuaternion expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        FixedQuaternion rotation = Subject.Rotation;

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.RepresentsSameRotation(rotation, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed4x4 matrix rotation to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, rotation);

        return new AndConstraint<Fixed4x4Assertions>(this);
    }

    /// <summary>Asserts that the first three entries of each of the first three rows form a vector of approximately unit length.</summary>
    /// <param name="tolerance">The inclusive absolute deviation from unit length. Use a nonnegative value. <see langword="null"/> uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <remarks>This checks basis lengths only. The rows may be nonorthogonal.</remarks>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    public AndConstraint<Fixed4x4Assertions> HaveNormalizedRotationBasis(
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.HasUnitAxes(Subject, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed4x4 matrix rotation basis to be normalized within +/- {0}{reason}, but found {1}.", limit, Subject);

        return new AndConstraint<Fixed4x4Assertions>(this);
    }
}
