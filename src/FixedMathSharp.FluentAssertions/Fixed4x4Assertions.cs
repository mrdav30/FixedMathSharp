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

public sealed class Fixed4x4Assertions : FixedStructAssertions<Fixed4x4, Fixed4x4Assertions>
{
    public Fixed4x4Assertions(Fixed4x4 value) : base(value) { }

    protected override string Identifier => "fixed4x4 matrix";

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

    public AndConstraint<Fixed4x4Assertions> BeIdentity(
        string because = "",
        params object[] becauseArgs)
    {
        return Be(Fixed4x4.Identity, because, becauseArgs);
    }

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
