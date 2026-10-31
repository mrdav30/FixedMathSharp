//=======================================================================
// FixedAssertionHelpers.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

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
        return (actual.X - expected.X).Abs() <= tolerance
            && (actual.Y - expected.Y).Abs() <= tolerance;
    }

    public static bool AreComponentApproximatelyEqual(Vector3d actual, Vector3d expected, Fixed64 tolerance)
    {
        return (actual.X - expected.X).Abs() <= tolerance
            && (actual.Y - expected.Y).Abs() <= tolerance
            && (actual.Z - expected.Z).Abs() <= tolerance;
    }

    public static bool AreComponentApproximatelyEqual(FixedQuaternion actual, FixedQuaternion expected, Fixed64 tolerance)
    {
        return (actual.X - expected.X).Abs() <= tolerance
            && (actual.Y - expected.Y).Abs() <= tolerance
            && (actual.Z - expected.Z).Abs() <= tolerance
            && (actual.W - expected.W).Abs() <= tolerance;
    }

    public static bool AreComponentApproximatelyEqual(Fixed3x3 actual, Fixed3x3 expected, Fixed64 tolerance)
    {
        return (actual.M11 - expected.M11).Abs() <= tolerance
            && (actual.M12 - expected.M12).Abs() <= tolerance
            && (actual.M13 - expected.M13).Abs() <= tolerance
            && (actual.M21 - expected.M21).Abs() <= tolerance
            && (actual.M22 - expected.M22).Abs() <= tolerance
            && (actual.M23 - expected.M23).Abs() <= tolerance
            && (actual.M31 - expected.M31).Abs() <= tolerance
            && (actual.M32 - expected.M32).Abs() <= tolerance
            && (actual.M33 - expected.M33).Abs() <= tolerance;
    }

    public static bool AreComponentApproximatelyEqual(Fixed4x4 actual, Fixed4x4 expected, Fixed64 tolerance)
    {
        return (actual.M11 - expected.M11).Abs() <= tolerance
            && (actual.M12 - expected.M12).Abs() <= tolerance
            && (actual.M13 - expected.M13).Abs() <= tolerance
            && (actual.M14 - expected.M14).Abs() <= tolerance
            && (actual.M21 - expected.M21).Abs() <= tolerance
            && (actual.M22 - expected.M22).Abs() <= tolerance
            && (actual.M23 - expected.M23).Abs() <= tolerance
            && (actual.M24 - expected.M24).Abs() <= tolerance
            && (actual.M31 - expected.M31).Abs() <= tolerance
            && (actual.M32 - expected.M32).Abs() <= tolerance
            && (actual.M33 - expected.M33).Abs() <= tolerance
            && (actual.M34 - expected.M34).Abs() <= tolerance
            && (actual.M41 - expected.M41).Abs() <= tolerance
            && (actual.M42 - expected.M42).Abs() <= tolerance
            && (actual.M43 - expected.M43).Abs() <= tolerance
            && (actual.M44 - expected.M44).Abs() <= tolerance;
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
        return AreApproximatelyEqual(new Vector3d(matrix.M11, matrix.M12, matrix.M13).Magnitude, Fixed64.One, tolerance)
            && AreApproximatelyEqual(new Vector3d(matrix.M21, matrix.M22, matrix.M23).Magnitude, Fixed64.One, tolerance)
            && AreApproximatelyEqual(new Vector3d(matrix.M31, matrix.M32, matrix.M33).Magnitude, Fixed64.One, tolerance);
    }

    public static bool HasUnitAxes(Fixed4x4 matrix, Fixed64 tolerance)
    {
        return AreApproximatelyEqual(new Vector3d(matrix.M11, matrix.M12, matrix.M13).Magnitude, Fixed64.One, tolerance)
            && AreApproximatelyEqual(new Vector3d(matrix.M21, matrix.M22, matrix.M23).Magnitude, Fixed64.One, tolerance)
            && AreApproximatelyEqual(new Vector3d(matrix.M31, matrix.M32, matrix.M33).Magnitude, Fixed64.One, tolerance);
    }
}
