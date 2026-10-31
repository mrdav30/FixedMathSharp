//=======================================================================
// Vector3dAssertions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

public sealed class Vector3dAssertions : ComparableTypeAssertions<Vector3d, Vector3dAssertions>
{
    public Vector3dAssertions(Vector3d value) : base(value, AssertionChain.GetOrCreate()) { }

    protected override string Identifier => "vector3d";

    public AndConstraint<Vector3dAssertions> BeApproximately(
        Vector3d expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector3d actual = (Vector3d)Subject;

        CurrentAssertionChain
            .ForCondition((actual - expected).Magnitude <= limit)
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected vector3d to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, actual);

        return new AndConstraint<Vector3dAssertions>(this);
    }

    public AndConstraint<Vector3dAssertions> NotBeApproximately(
        Vector3d expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector3d actual = (Vector3d)Subject;

        CurrentAssertionChain
            .ForCondition((actual - expected).Magnitude > limit)
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected vector3d not to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, actual);

        return new AndConstraint<Vector3dAssertions>(this);
    }

    public AndConstraint<Vector3dAssertions> HaveComponentApproximately(
        Vector3d expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector3d actual = (Vector3d)Subject;

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreComponentApproximatelyEqual(actual, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected vector3d components to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, actual);

        return new AndConstraint<Vector3dAssertions>(this);
    }

    public AndConstraint<Vector3dAssertions> HaveMagnitudeApproximately(
        Fixed64 expected,
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector3d actual = (Vector3d)Subject;

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreApproximatelyEqual(actual.Magnitude, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected vector3d magnitude to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, actual.Magnitude);

        return new AndConstraint<Vector3dAssertions>(this);
    }

    public AndConstraint<Vector3dAssertions> BeNormalized(
        Fixed64? tolerance = null,
        string because = "",
        params object[] becauseArgs)
    {
        Fixed64 limit = FixedAssertionHelpers.ResolveTolerance(tolerance);
        Vector3d actual = (Vector3d)Subject;

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreApproximatelyEqual(actual.Magnitude, Fixed64.One, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected vector3d to be normalized within +/- {0}{reason}, but found magnitude {1}.", limit, actual.Magnitude);

        return new AndConstraint<Vector3dAssertions>(this);
    }
}
