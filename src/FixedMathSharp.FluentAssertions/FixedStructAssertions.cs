//=======================================================================
// FixedStructAssertions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;

namespace FixedMathSharp.Assertions;

/// <summary>Provides equality assertions and a shared assertion chain for fixed-point values.</summary>
/// <typeparam name="TSubject">The type of the value under assertion.</typeparam>
/// <typeparam name="TAssertions">The concrete assertions type returned when chaining assertions. Derived classes must be assignable to this type.</typeparam>
/// <remarks>Failures use the active FluentAssertions test framework and assertion scope.</remarks>
public abstract class FixedStructAssertions<TSubject, TAssertions>
    where TAssertions : FixedStructAssertions<TSubject, TAssertions>
{
    /// <summary>Stores the subject and acquires the assertion chain for the current FluentAssertions context.</summary>
    /// <param name="value">The subject to retain for subsequent assertions. A value type is stored by value.</param>
    protected FixedStructAssertions(TSubject value)
    {
        Subject = value;
        CurrentAssertionChain = AssertionChain.GetOrCreate();
    }

    /// <summary>Gets the subject evaluated by this assertion object.</summary>
    /// <value>The subject supplied when constructing the assertion object.</value>
    protected TSubject Subject { get; }

    /// <summary>Gets the FluentAssertions chain used to evaluate this assertion.</summary>
    /// <value>The chain acquired at construction for reporting and chaining assertions.</value>
    protected AssertionChain CurrentAssertionChain { get; }

    /// <summary>Gets the label used to identify this kind of subject in failure messages.</summary>
    /// <value>The display name of the subject type.</value>
    protected abstract string Identifier { get; }

    /// <summary>Asserts that the subject equals the expected value according to <see cref="object.Equals(object, object)"/>.</summary>
    /// <param name="expected">The value the subject must equal.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    /// <exception cref="System.InvalidCastException">The assertion object is incompatible with <typeparamref name="TAssertions"/>.</exception>
    public AndConstraint<TAssertions> Be(TSubject expected, string because = "", params object[] becauseArgs)
    {
        CurrentAssertionChain
            .ForCondition(Equals(Subject, expected))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected " + Identifier + " to equal {0}{reason}, but found {1}.", expected, Subject);

        return new AndConstraint<TAssertions>((TAssertions)this);
    }

    /// <summary>Asserts that the subject differs from the supplied value according to <see cref="object.Equals(object, object)"/>.</summary>
    /// <param name="unexpected">The value the subject must differ from.</param>
    /// <param name="because">An optional format string that explains the reason for the assertion.</param>
    /// <param name="becauseArgs">Values inserted into the placeholders in <paramref name="because"/>.</param>
    /// <returns>A constraint for chaining more assertions on the same subject.</returns>
    /// <exception cref="AssertionFailedException">The check fails and FluentAssertions uses its default assertion exception. An active assertion scope can collect the failure instead.</exception>
    /// <exception cref="System.InvalidCastException">The assertion object is incompatible with <typeparamref name="TAssertions"/>.</exception>
    public AndConstraint<TAssertions> NotBe(TSubject unexpected, string because = "", params object[] becauseArgs)
    {
        CurrentAssertionChain
            .ForCondition(!Equals(Subject, unexpected))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected " + Identifier + " not to equal {0}{reason}, but found {1}.", unexpected, Subject);

        return new AndConstraint<TAssertions>((TAssertions)this);
    }
}
