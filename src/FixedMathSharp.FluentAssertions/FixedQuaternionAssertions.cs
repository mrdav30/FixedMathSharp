//=======================================================================
// FixedQuaternionAssertions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

public sealed class FixedQuaternionAssertions : FixedStructAssertions<FixedQuaternion, FixedQuaternionAssertions>
{
    public FixedQuaternionAssertions(FixedQuaternion value) : base(value) { }

    protected override string Identifier => "fixed quaternion";

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

    public AndConstraint<FixedQuaternionAssertions> HaveComponentApproximately(
        FixedQuaternion expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        return BeApproximately(expected, tolerance, because, becauseArgs);
    }

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

    public AndConstraint<FixedQuaternionAssertions> BeIdentity(
        string because = "",
        params object[] becauseArgs)
    {
        return Be(FixedQuaternion.Identity, because, becauseArgs);
    }

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
