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

public sealed class Vector2dAssertions : ComparableTypeAssertions<Vector2d, Vector2dAssertions>
{
    public Vector2dAssertions(Vector2d value) : base(value, AssertionChain.GetOrCreate()) { }

    protected override string Identifier => "vector2d";

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
