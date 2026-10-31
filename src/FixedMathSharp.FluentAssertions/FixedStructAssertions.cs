//=======================================================================
// FixedStructAssertions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

public abstract class FixedStructAssertions<TSubject, TAssertions>
    where TAssertions : FixedStructAssertions<TSubject, TAssertions>
{
    protected FixedStructAssertions(TSubject value)
    {
        Subject = value;
        CurrentAssertionChain = AssertionChain.GetOrCreate();
    }

    protected TSubject Subject { get; }

    protected AssertionChain CurrentAssertionChain { get; }

    protected abstract string Identifier { get; }

    public AndConstraint<TAssertions> Be(TSubject expected, string because = "", params object[] becauseArgs)
    {
        CurrentAssertionChain
            .ForCondition(Equals(Subject, expected))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected " + Identifier + " to equal {0}{reason}, but found {1}.", expected, Subject);

        return new AndConstraint<TAssertions>((TAssertions)this);
    }

    public AndConstraint<TAssertions> NotBe(TSubject unexpected, string because = "", params object[] becauseArgs)
    {
        CurrentAssertionChain
            .ForCondition(!Equals(Subject, unexpected))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected " + Identifier + " not to equal {0}{reason}, but found {1}.", unexpected, Subject);

        return new AndConstraint<TAssertions>((TAssertions)this);
    }
}
