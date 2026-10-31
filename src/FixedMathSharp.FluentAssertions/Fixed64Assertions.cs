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

public sealed class Fixed64Assertions : ComparableTypeAssertions<Fixed64, Fixed64Assertions>
{
    public Fixed64Assertions(Fixed64 value) : base(value, AssertionChain.GetOrCreate()) { }

    protected override string Identifier => "fixed64";

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
