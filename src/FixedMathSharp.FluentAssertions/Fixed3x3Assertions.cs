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

public sealed class Fixed3x3Assertions : FixedStructAssertions<Fixed3x3, Fixed3x3Assertions>
{
    public Fixed3x3Assertions(Fixed3x3 value) : base(value) { }

    protected override string Identifier => "fixed3x3 matrix";

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

    public AndConstraint<Fixed3x3Assertions> BeIdentity(
        string because = "",
        params object[] becauseArgs)
    {
        return Be(Fixed3x3.Identity, because, becauseArgs);
    }

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
