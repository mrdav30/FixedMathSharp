using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

internal static class FixedAssertionHelpers
{
    public static Fixed64 ResolveTolerance(Fixed64? tolerance)
    {
        return tolerance ?? Fixed64.Epsilon;
    }

    public static bool AreApproximatelyEqual(Fixed64 actual, Fixed64 expected, Fixed64 tolerance)
    {
        return (actual - expected).Abs() <= tolerance;
    }

    public static bool AreComponentApproximatelyEqual(Vector2d actual, Vector2d expected, Fixed64 tolerance)
    {
        return (actual.x - expected.x).Abs() <= tolerance
            && (actual.y - expected.y).Abs() <= tolerance;
    }

    public static bool AreComponentApproximatelyEqual(Vector3d actual, Vector3d expected, Fixed64 tolerance)
    {
        return (actual.x - expected.x).Abs() <= tolerance
            && (actual.y - expected.y).Abs() <= tolerance
            && (actual.z - expected.z).Abs() <= tolerance;
    }

    public static bool AreComponentApproximatelyEqual(FixedQuaternion actual, FixedQuaternion expected, Fixed64 tolerance)
    {
        return (actual.x - expected.x).Abs() <= tolerance
            && (actual.y - expected.y).Abs() <= tolerance
            && (actual.z - expected.z).Abs() <= tolerance
            && (actual.w - expected.w).Abs() <= tolerance;
    }

    public static bool AreComponentApproximatelyEqual(Fixed3x3 actual, Fixed3x3 expected, Fixed64 tolerance)
    {
        return (actual.m00 - expected.m00).Abs() <= tolerance
            && (actual.m01 - expected.m01).Abs() <= tolerance
            && (actual.m02 - expected.m02).Abs() <= tolerance
            && (actual.m10 - expected.m10).Abs() <= tolerance
            && (actual.m11 - expected.m11).Abs() <= tolerance
            && (actual.m12 - expected.m12).Abs() <= tolerance
            && (actual.m20 - expected.m20).Abs() <= tolerance
            && (actual.m21 - expected.m21).Abs() <= tolerance
            && (actual.m22 - expected.m22).Abs() <= tolerance;
    }

    public static bool AreComponentApproximatelyEqual(Fixed4x4 actual, Fixed4x4 expected, Fixed64 tolerance)
    {
        return (actual.m00 - expected.m00).Abs() <= tolerance
            && (actual.m01 - expected.m01).Abs() <= tolerance
            && (actual.m02 - expected.m02).Abs() <= tolerance
            && (actual.m03 - expected.m03).Abs() <= tolerance
            && (actual.m10 - expected.m10).Abs() <= tolerance
            && (actual.m11 - expected.m11).Abs() <= tolerance
            && (actual.m12 - expected.m12).Abs() <= tolerance
            && (actual.m13 - expected.m13).Abs() <= tolerance
            && (actual.m20 - expected.m20).Abs() <= tolerance
            && (actual.m21 - expected.m21).Abs() <= tolerance
            && (actual.m22 - expected.m22).Abs() <= tolerance
            && (actual.m23 - expected.m23).Abs() <= tolerance
            && (actual.m30 - expected.m30).Abs() <= tolerance
            && (actual.m31 - expected.m31).Abs() <= tolerance
            && (actual.m32 - expected.m32).Abs() <= tolerance
            && (actual.m33 - expected.m33).Abs() <= tolerance;
    }

    public static bool RepresentsSameRotation(FixedQuaternion actual, FixedQuaternion expected, Fixed64 tolerance)
    {
        FixedQuaternion normalizedActual = FixedQuaternion.GetNormalized(actual);
        FixedQuaternion normalizedExpected = FixedQuaternion.GetNormalized(expected);
        Fixed64 alignment = FixedMath.Abs(FixedQuaternion.Dot(normalizedActual, normalizedExpected));
        return FixedMath.Abs(Fixed64.One - alignment) <= tolerance;
    }

    public static bool HasUnitAxes(Fixed3x3 matrix, Fixed64 tolerance)
    {
        return AreApproximatelyEqual(new Vector3d(matrix.m00, matrix.m01, matrix.m02).Magnitude, Fixed64.One, tolerance)
            && AreApproximatelyEqual(new Vector3d(matrix.m10, matrix.m11, matrix.m12).Magnitude, Fixed64.One, tolerance)
            && AreApproximatelyEqual(new Vector3d(matrix.m20, matrix.m21, matrix.m22).Magnitude, Fixed64.One, tolerance);
    }

    public static bool HasUnitAxes(Fixed4x4 matrix, Fixed64 tolerance)
    {
        return AreApproximatelyEqual(new Vector3d(matrix.m00, matrix.m01, matrix.m02).Magnitude, Fixed64.One, tolerance)
            && AreApproximatelyEqual(new Vector3d(matrix.m10, matrix.m11, matrix.m12).Magnitude, Fixed64.One, tolerance)
            && AreApproximatelyEqual(new Vector3d(matrix.m20, matrix.m21, matrix.m22).Magnitude, Fixed64.One, tolerance);
    }
}

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

public static class Fixed64AssertionsExtensions
{
    public static Fixed64Assertions Should(this Fixed64 actualValue)
    {
        return new Fixed64Assertions(actualValue);
    }
}

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

public static class Vector2dAssertionsExtensions
{
    public static Vector2dAssertions Should(this Vector2d actualValue)
    {
        return new Vector2dAssertions(actualValue);
    }
}

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

public static class Vector3dAssertionsExtensions
{
    public static Vector3dAssertions Should(this Vector3d actualValue)
    {
        return new Vector3dAssertions(actualValue);
    }
}

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

public static class FixedQuaternionAssertionsExtensions
{
    public static FixedQuaternionAssertions Should(this FixedQuaternion actualValue)
    {
        return new FixedQuaternionAssertions(actualValue);
    }
}

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
        Vector3d scale = Fixed3x3.ExtractScale(Subject);

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreComponentApproximatelyEqual(scale, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed3x3 matrix scale to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, scale);

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

public static class Fixed3x3AssertionsExtensions
{
    public static Fixed3x3Assertions Should(this Fixed3x3 actualValue)
    {
        return new Fixed3x3Assertions(actualValue);
    }
}

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
        Vector3d scale = Subject.Scale;

        CurrentAssertionChain
            .ForCondition(FixedAssertionHelpers.AreComponentApproximatelyEqual(scale, expected, limit))
            .BecauseOf(because, becauseArgs)
            .FailWith("Expected fixed4x4 matrix scale to be approximately {0} +/- {1}{reason}, but found {2}.", expected, limit, scale);

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

public static class Fixed4x4AssertionsExtensions
{
    public static Fixed4x4Assertions Should(this Fixed4x4 actualValue)
    {
        return new Fixed4x4Assertions(actualValue);
    }
}
