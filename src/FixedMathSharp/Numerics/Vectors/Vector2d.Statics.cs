//=======================================================================
// Vector2d.Statics.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using FixedMathSharp.Geometry;

namespace FixedMathSharp;

/// <content>
/// Static utility methods for <see cref="Vector2d"/>, including arithmetic helpers
/// and safe (non-saturating) operation variants.
/// </content>
public partial struct Vector2d
{
    /// <summary>
    /// Adds two vectors component-wise.
    /// </summary>
    /// <param name="v1">The first vector to add.</param>
    /// <param name="v2">The second vector to add.</param>
    /// <returns>The component-wise sum, with each component saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Add(Vector2d v1, Vector2d v2) => v1 + v2;

    /// <summary>
    /// Attempts to add two vectors without component saturation.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <param name="result">
    /// The exact component-wise sum when every component is representable; otherwise, <see langword="default"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when every exact component is representable; otherwise, <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryAdd(Vector2d left, Vector2d right, out Vector2d result)
    {
        if (!Fixed64.TryAdd(left.X, right.X, out Fixed64 x)
            || !Fixed64.TryAdd(left.Y, right.Y, out Fixed64 y))
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    /// <summary>
    /// Subtracts two vectors component-wise.
    /// </summary>
    /// <param name="v1">The vector to subtract from.</param>
    /// <param name="v2">The vector to subtract.</param>
    /// <returns>The component-wise difference, with each component saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Subtract(Vector2d v1, Vector2d v2) => v1 - v2;

    /// <summary>
    /// Attempts to subtract two vectors without component saturation.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <param name="result">
    /// The exact component-wise difference when every component is representable; otherwise, <see langword="default"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when every exact component is representable; otherwise, <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TrySubtract(Vector2d left, Vector2d right, out Vector2d result)
    {
        if (!Fixed64.TrySubtract(left.X, right.X, out Fixed64 x)
            || !Fixed64.TrySubtract(left.Y, right.Y, out Fixed64 y))
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    /// <summary>
    /// Attempts to add two vectors and subtract a third component-wise without intermediate saturation.
    /// </summary>
    /// <param name="firstAddend">The first addend.</param>
    /// <param name="secondAddend">The second addend.</param>
    /// <param name="subtrahend">The vector to subtract from the exact component-wise sum.</param>
    /// <param name="result">
    /// The exact component-wise result when every component is representable; otherwise, <see langword="default"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when every exact component is representable; otherwise, <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryAddSubtract(
        Vector2d firstAddend,
        Vector2d secondAddend,
        Vector2d subtrahend,
        out Vector2d result)
    {
        if (!Fixed64.TryAddSubtract(firstAddend.X, secondAddend.X, subtrahend.X, out Fixed64 x)
            || !Fixed64.TryAddSubtract(firstAddend.Y, secondAddend.Y, subtrahend.Y, out Fixed64 y))
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    /// <summary>
    /// Attempts to compute <c>(firstLeft + firstRight) -
    /// (secondLeft + secondRight)</c> component-wise without intermediate
    /// saturation.
    /// </summary>
    /// <param name="firstLeft">The first addend of the minuend sum.</param>
    /// <param name="firstRight">The second addend of the minuend sum.</param>
    /// <param name="secondLeft">The first addend of the subtrahend sum.</param>
    /// <param name="secondRight">The second addend of the subtrahend sum.</param>
    /// <param name="result">The difference of the two sums on success, or the zero vector on failure.</param>
    /// <returns><see langword="true"/> if all final components are representable, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TrySubtractSums(
        Vector2d firstLeft,
        Vector2d firstRight,
        Vector2d secondLeft,
        Vector2d secondRight,
        out Vector2d result)
    {
        bool representable = Fixed64.TrySubtractSums(
                firstLeft.X,
                firstRight.X,
                secondLeft.X,
                secondRight.X,
                out Fixed64 x)
            & Fixed64.TrySubtractSums(
                firstLeft.Y,
                firstRight.Y,
                secondLeft.Y,
                secondRight.Y,
                out Fixed64 y);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    /// <summary>
    /// Compares the exact component-difference projections of two vectors.
    /// </summary>
    /// <param name="candidate">The candidate point.</param>
    /// <param name="current">The point to compare against.</param>
    /// <param name="direction">The projection direction; it need not be normalized.</param>
    /// <returns>
    /// A negative value, zero, or a positive value when the candidate projection is
    /// respectively less than, equal to, or greater than the current projection.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CompareProjection(Vector2d candidate, Vector2d current, Vector2d direction) =>
        WideGeometry.CompareDifferenceProjection(
            candidate.X,
            current.X,
            direction.X,
            candidate.Y,
            current.Y,
            direction.Y,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero);

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    /// <param name="v1">The first factor.</param>
    /// <param name="v2">The second factor.</param>
    /// <returns>The component-wise product, with each component saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Multiply(Vector2d v1, Vector2d v2) => v1 * v2;

    /// <summary>
    /// Multiplies each vector component by the specified scalar.
    /// </summary>
    /// <param name="value">The vector to scale.</param>
    /// <param name="factor">The scalar factor.</param>
    /// <returns>The component-wise product, with each component saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Multiply(Vector2d value, Fixed64 factor) => value * factor;

    /// <summary>
    /// Divides each component of the first vector by the corresponding component of the second vector.
    /// </summary>
    /// <param name="v1">The numerator.</param>
    /// <param name="v2">The divisor.</param>
    /// <returns>The component-wise quotient, with each component saturated at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException">A divisor component is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Divide(Vector2d v1, Vector2d v2) => new(v1.X / v2.X, v1.Y / v2.Y);

    /// <summary>
    /// Divides each vector component by the specified scalar.
    /// </summary>
    /// <param name="value">The vector to scale.</param>
    /// <param name="divisor">The scalar divisor.</param>
    /// <returns>The component-wise quotient, with each component saturated at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException">The divisor is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Divide(Vector2d value, Fixed64 divisor) => value / divisor;

    /// <summary>
    /// Clamps each component of the given vector within the specified min and max bounds.
    /// </summary>
    /// <param name="value">The vector to clamp.</param>
    /// <param name="min">The inclusive lower limits. Each must be at most its matching upper limit.</param>
    /// <param name="max">The inclusive upper limits.</param>
    /// <returns>A copy with each component limited to its corresponding interval.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Clamp(Vector2d value, Vector2d min, Vector2d max) =>
        new(FixedMath.Clamp(value.X, min.X, max.X),
            FixedMath.Clamp(value.Y, min.Y, max.Y));

    /// <summary>
    /// Normalizes the given vector, returning a unit vector with the same direction.
    /// </summary>
    /// <param name="value">The vector to normalize.</param>
    /// <returns>A normalized (unit) vector with the same direction.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d GetNormalized(Vector2d value)
    {
        bool magnitudeIsRepresentable = TryGetMagnitude(
            value,
            out Fixed64 mag,
            out bool isNormalized);

        if (mag == Fixed64.Zero)
            return new Vector2d(Fixed64.Zero, Fixed64.Zero);

        if (isNormalized)
            return value;

        if (!magnitudeIsRepresentable || mag == Fixed64.One)
            return WideNormalization.GetNormalized(value);

        if (mag <= FixedMath.ScaleSafeMagnitudeThreshold)
            return GetScaleNormalized(value);

        var normalized = new Vector2d(
            FixedMath.FastDiv(value.X, mag),
            FixedMath.FastDiv(value.Y, mag));
        return normalized.IsNormalized()
            ? normalized
            : WideNormalization.GetNormalized(value);
    }

    internal static Vector2d GetScaleNormalized(Vector2d value)
    {
        Fixed64 scale = FixedMath.Max(value.X.Abs(), value.Y.Abs());
        Vector2d scaled = value / scale;
        // The dominant component rounds to +/-One, even when Abs(MinValue)
        // saturates to MaxValue: the ratio excess is less than half a raw unit.
        // The squared sum is bounded by Two, so a second rescaling is redundant.
        Fixed64 scaledMagnitude = FixedMath.Sqrt(scaled.X * scaled.X + scaled.Y * scaled.Y);
        return scaled / scaledMagnitude;
    }

    /// <summary>
    /// Attempts to transform a component-scaled local point by a rotation and
    /// world origin with one final round-half-to-even conversion per
    /// component.
    /// </summary>
    /// <param name="origin">The origin of the local frame, expressed in world coordinates.</param>
    /// <param name="localPoint">The point in local coordinates.</param>
    /// <param name="scale">The per-axis local scale factors.</param>
    /// <param name="angleInRadians">The local-to-world rotation in radians, positive from +X toward +Y.</param>
    /// <param name="result">The rounded point in world coordinates on success, or the zero vector on failure.</param>
    /// <returns><see langword="true"/> if all final components are representable, otherwise <see langword="false"/>.</returns>
    public static bool TryTransformScaledPoint(
        Vector2d origin,
        Vector2d localPoint,
        Vector2d scale,
        Fixed64 angleInRadians,
        out Vector2d result) =>
        TryTransformScaledPoint(
            origin,
            localPoint,
            scale,
            Vector2d.Zero,
            angleInRadians,
            out result);

    /// <summary>
    /// Attempts to transform a component-scaled local point plus an unscaled
    /// local displacement by a rotation and world origin with one final
    /// round-half-to-even conversion per component.
    /// </summary>
    /// <remarks>
    /// Computes
    /// <c>origin + Rotate(scale * localPoint + localDisplacement)</c> without
    /// narrowing the scaled point, local sum, or rotated offset independently.
    /// </remarks>
    /// <param name="origin">The origin of the local frame, expressed in world coordinates.</param>
    /// <param name="localPoint">The point in local coordinates.</param>
    /// <param name="scale">The per-axis local scale factors.</param>
    /// <param name="localDisplacement">The local offset added after applying the scale and before rotation.</param>
    /// <param name="angleInRadians">The local-to-world rotation in radians, positive from +X toward +Y.</param>
    /// <param name="result">The rounded point in world coordinates on success, or the zero vector on failure.</param>
    /// <returns><see langword="true"/> if all final components are representable, otherwise <see langword="false"/>.</returns>
    public static bool TryTransformScaledPoint(
        Vector2d origin,
        Vector2d localPoint,
        Vector2d scale,
        Vector2d localDisplacement,
        Fixed64 angleInRadians,
        out Vector2d result) =>
        WideVector2dTransform.TryTransformScaledPoint(
            origin,
            localPoint,
            scale,
            localDisplacement,
            angleInRadians,
            out result);

    /// <summary>
    /// Attempts to transform a world point into a component-scaled local frame
    /// with one final round-half-to-even conversion per component.
    /// </summary>
    /// <remarks>
    /// Computes <c>InverseRotate(worldPoint - origin) / scale</c> without
    /// narrowing the world offset, rotated projection, or scale division
    /// independently. A zero scale component or unrepresentable final
    /// coordinate returns false and a zero result.
    /// </remarks>
    /// <param name="origin">The origin of the local frame, expressed in world coordinates.</param>
    /// <param name="worldPoint">The point to map from world coordinates.</param>
    /// <param name="scale">The nonzero per-axis local scale factors to undo.</param>
    /// <param name="angleInRadians">The local-to-world rotation in radians, positive from +X toward +Y.</param>
    /// <param name="localPoint">The point in local coordinates on success, or the zero vector on failure.</param>
    /// <returns><see langword="true"/> if each scale factor is nonzero and both final coordinates are representable, otherwise <see langword="false"/>.</returns>
    public static bool TryInverseTransformScaledPoint(
        Vector2d origin,
        Vector2d worldPoint,
        Vector2d scale,
        Fixed64 angleInRadians,
        out Vector2d localPoint) =>
        WideVector2dTransform.TryInverseTransformScaledPoint(
            origin,
            worldPoint,
            scale,
            angleInRadians,
            out localPoint);

    /// <summary>
    /// Attempts to compose two component-scaled offsets in a shared frame and
    /// one rotated inner-frame displacement with one final round-half-to-even
    /// conversion per component.
    /// </summary>
    /// <remarks>
    /// Computes
    /// <c>outerScale * outerLocalPoint
    /// + innerFrameScale * innerFrameOffset
    /// + Rotate(innerLocalDisplacement)</c>
    /// without narrowing either scaled offset or the rotated displacement
    /// independently.
    /// </remarks>
    /// <param name="outerLocalPoint">The first offset, expressed along the shared frame axes.</param>
    /// <param name="outerScale">The per-axis scale factors for the first offset.</param>
    /// <param name="innerFrameOffset">The second offset, expressed along the shared frame axes.</param>
    /// <param name="innerFrameScale">The per-axis scale factors for the second offset.</param>
    /// <param name="innerLocalDisplacement">The displacement in the inner frame. It is rotated directly, without either scale.</param>
    /// <param name="result">The composed offset in the shared frame on success, or the zero vector on failure.</param>
    /// <param name="innerAngleInRadians">The inner-to-shared angle in radians, positive from +X toward +Y.</param>
    /// <returns><see langword="true"/> if all final components are representable, otherwise <see langword="false"/>.</returns>
    public static bool TryComposeScaledLocalPoints(
        Vector2d outerLocalPoint,
        Vector2d outerScale,
        Vector2d innerFrameOffset,
        Vector2d innerFrameScale,
        Vector2d innerLocalDisplacement,
        Fixed64 innerAngleInRadians,
        out Vector2d result) =>
        WideVector2dTransform.TryComposeScaledLocalPoints(
            outerLocalPoint,
            outerScale,
            innerFrameOffset,
            innerFrameScale,
            innerLocalDisplacement,
            innerAngleInRadians,
            out result);

    /// <summary>
    /// Returns the normalized direction from <paramref name="start"/> toward
    /// <paramref name="end"/> across the complete coordinate domain.
    /// </summary>
    /// <remarks>
    /// Equal endpoints return <see cref="Zero"/>. Endpoint differences are
    /// evaluated exactly even when a component cannot be represented by
    /// <see cref="Fixed64"/>.
    /// </remarks>
    /// <param name="start">The starting point.</param>
    /// <param name="end">The ending point in the same coordinate space.</param>
    /// <returns>The approximate unit direction from start to end, or zero for coincident points.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d GetDirection(Vector2d start, Vector2d end) =>
        WideNormalization.GetDirection(start, end);

    /// <summary>
    /// Returns the magnitude (length) of the given vector.
    /// </summary>
    /// <param name="vector">The vector to compute the magnitude of.</param>
    /// <returns>The magnitude (length) of the vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 GetMagnitude(Vector2d vector)
    {
        _ = TryGetMagnitude(vector, out Fixed64 magnitude);
        return magnitude;
    }

    /// <summary>
    /// Attempts to return the magnitude of the given vector without saturating the result.
    /// </summary>
    /// <param name="vector">The vector to measure.</param>
    /// <param name="magnitude">The magnitude, or <see cref="Fixed64.MaxValue"/> when it is not representable.</param>
    /// <returns><see langword="true"/> when the magnitude fits in <see cref="Fixed64"/>; otherwise, <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryGetMagnitude(Vector2d vector, out Fixed64 magnitude) =>
        TryGetMagnitude(vector, out magnitude, out _);

    private static bool TryGetMagnitude(
        Vector2d vector,
        out Fixed64 magnitude,
        out bool isNormalized)
    {
        Fixed64 mag = (vector.X * vector.X) + (vector.Y * vector.Y);
        isNormalized = mag != Fixed64.Zero
            && FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon;

        if (mag == Fixed64.MaxValue)
            return FixedMath.TryGetScaledMagnitude(
                vector.X,
                vector.Y,
                Fixed64.Zero,
                Fixed64.Zero,
                out magnitude);

        if (mag <= FixedMath.ScaleSafeMagnitudeSquaredThreshold)
        {
            magnitude = FixedMath.GetScaledMagnitude(
                vector.X,
                vector.Y,
                Fixed64.Zero,
                Fixed64.Zero);
            return true;
        }

        if (isNormalized)
        {
            magnitude = Fixed64.One;
            return true;
        }

        magnitude = FixedMath.Sqrt(mag);
        return true;
    }

    /// <summary>
    /// Attempts to return the distance between two endpoints without saturating
    /// either component difference.
    /// </summary>
    /// <param name="start">The first endpoint.</param>
    /// <param name="end">The second endpoint.</param>
    /// <param name="distance">The rounded distance, or <see cref="Fixed64.MaxValue"/> when it is not representable.</param>
    /// <returns><see langword="true"/> when the distance fits in <see cref="Fixed64"/>; otherwise, <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryGetDistance(Vector2d start, Vector2d end, out Fixed64 distance) =>
        WideGeometry.TryGetDistance(start, end, out distance);

    /// <summary>
    /// Compares the exact squared magnitudes of two vectors without fixed-point saturation.
    /// </summary>
    /// <returns>A negative value when <paramref name="left"/> is shorter, zero when the
    /// magnitudes are equal, or a positive value when <paramref name="left"/> is longer.</returns>
    /// <param name="left">The first vector.</param>
    /// <param name="right">The second vector.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CompareMagnitudeSquared(Vector2d left, Vector2d right) =>
        Fixed64.CompareMagnitudeSquared(
            left.X,
            left.Y,
            Fixed64.Zero,
            Fixed64.Zero,
            right.X,
            right.Y,
            Fixed64.Zero,
            Fixed64.Zero);

    /// <summary>
    /// Returns a new <see cref="Vector2d"/> where each component is the absolute value of the corresponding input component.
    /// </summary>
    /// <param name="value">The input vector.</param>
    /// <returns>A vector with absolute values for each component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Abs(Vector2d value) => new(value.X.Abs(), value.Y.Abs());

    /// <summary>
    /// Returns a new <see cref="Vector2d"/> where each component is the sign of the corresponding input component.
    /// </summary>
    /// <param name="value">The input vector.</param>
    /// <returns>A vector where each component is -1, 0, or 1 based on the sign of the input.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Sign(Vector2d value) => new(value.X.Sign(), value.Y.Sign());

    /// <summary>
    /// Attempts to calculate the exact non-negative weighted average with one
    /// final round-half-to-even conversion per component.
    /// </summary>
    /// <param name="values">The vectors in a common coordinate space, paired by index with the weights.</param>
    /// <param name="weights">The matching nonnegative relative weights. A zero weight excludes its vector.</param>
    /// <param name="average">The weighted average on success, or the zero vector on failure.</param>
    /// <returns><see langword="true"/> if the total weight is positive, otherwise <see langword="false"/>.</returns>
    /// <remarks>The spans are read during the call. Each component is divided by the exact total weight and rounded once with ties to even.</remarks>
    /// <exception cref="ArgumentException">The spans have different lengths.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A weight is negative.</exception>
    public static bool TryGetWeightedAverage(
        ReadOnlySpan<Vector2d> values,
        ReadOnlySpan<Fixed64> weights,
        out Vector2d average)
    {
        WideWeightedAverage.ValidateInputs(values.Length, weights);
        return WideWeightedAverage.TryGet(values, weights, out average);
    }

    /// <summary>
    /// Calculates the arithmetic mean with one final round-half-to-even
    /// conversion per component.
    /// </summary>
    /// <param name="values">The non-empty sequence of vectors to average.</param>
    /// <returns>The arithmetic mean of <paramref name="values"/>.</returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="values"/> is empty.
    /// </exception>
    public static Vector2d GetAverage(ReadOnlySpan<Vector2d> values)
    {
        if (values.IsEmpty)
        {
            throw new ArgumentException(
                "At least one value is required.",
                nameof(values));
        }

        return WideWeightedAverage.GetAverage(values);
    }

    /// <summary>
    /// Creates a vector from a given angle in radians.
    /// </summary>
    /// <param name="angle">The angle in radians, positive from +X toward +Y.</param>
    /// <returns>The pair <c>(cos(angle), sin(angle))</c>, suitable for rotation.</returns>
    public static Vector2d CreateRotation(Fixed64 angle) => new(FixedMath.Cos(angle), FixedMath.Sin(angle));

    /// <summary>
    /// Linearly interpolates between two vectors.
    /// </summary>
    /// <param name="a">The starting vector.</param>
    /// <param name="b">The ending vector in the same coordinate space.</param>
    /// <param name="amount">The interpolation fraction, clamped to [0, 1].</param>
    /// <returns>The component-wise interpolated vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Lerp(Vector2d a, Vector2d b, Fixed64 amount)
    {
        amount = FixedMath.Clamp01(amount);
        return new Vector2d(
            FixedMath.Lerp(a.X, b.X, amount),
            FixedMath.Lerp(a.Y, b.Y, amount));
    }

    /// <summary>
    /// Calculates a position between three points using barycentric weights for the second and third vertices.
    /// </summary>
    /// <param name="coordA">The first vertex of the triangle.</param>
    /// <param name="coordB">The second vertex of the triangle.</param>
    /// <param name="coordC">The third vertex of the triangle.</param>
    /// <param name="weightB">The barycentric weight for the second vertex.</param>
    /// <param name="weightC">The barycentric weight for the third vertex.</param>
    /// <returns>The cartesian translation represented by the barycentric coordinates within the triangle.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d BarycentricCoordinates(
        Vector2d coordA,
        Vector2d coordB,
        Vector2d coordC,
        Fixed64 weightB,
        Fixed64 weightC)
    {
        return new(
            FixedMath.BarycentricCoordinate(coordA.X, coordB.X, coordC.X, weightB, weightC),
            FixedMath.BarycentricCoordinate(coordA.Y, coordB.Y, coordC.Y, weightB, weightC));
    }

    /// <summary>Computes the distance between two points.</summary>
    /// <param name="start">The first point.</param>
    /// <param name="end">The second point in the same coordinate space.</param>
    /// <returns>The Euclidean distance in the same units as the coordinates, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Distance(Vector2d start, Vector2d end) => start.Distance(end);

    /// <summary>Computes the squared distance between two points.</summary>
    /// <param name="start">The first point.</param>
    /// <param name="end">The second point in the same coordinate space.</param>
    /// <returns>The squared Euclidean distance in squared coordinate units, with saturation at <see cref="Fixed64.MaxValue"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 DistanceSquared(Vector2d start, Vector2d end) => start.DistanceSquared(end);

    /// <summary>
    /// Calculates the closest point on a finite line segment to a given point.
    /// </summary>
    /// <param name="point">The point to project onto the segment.</param>
    /// <param name="start">The start of the line segment.</param>
    /// <param name="end">The end of the line segment.</param>
    /// <returns>The closest point on the segment to the given point.</returns>
    /// <remarks>
    /// Zero-length segments deterministically return <paramref name="start"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ClosestPointOnLineSegment(Vector2d point, Vector2d start, Vector2d end)
    {
        Fixed64 t = GetClosestPointOnLineSegmentParameter(point, start, end);
        return new Vector2d(
            FixedMath.Lerp(start.X, end.X, t),
            FixedMath.Lerp(start.Y, end.Y, t));
    }

    internal static Fixed64 GetClosestPointOnLineSegmentParameter(
        Vector2d point,
        Vector2d start,
        Vector2d end)
    {
        Signed192 denominator = WideGeometry.GetDifferenceDotProduct2D(
            end.X, start.X, end.Y, start.Y,
            end.X, start.X, end.Y, start.Y);
        if (denominator.IsZero)
            return Fixed64.Zero;

        Signed192 numerator = WideGeometry.GetDifferenceDotProduct2D(
            point.X, start.X, point.Y, start.Y,
            end.X, start.X, end.Y, start.Y);
        if (numerator.Sign <= 0)
            return Fixed64.Zero;
        if (WideArithmetic.CompareMagnitude(numerator, denominator) >= 0)
            return Fixed64.One;

        _ = Fixed64.TryGetUnitIntervalRatio(numerator, denominator, out Fixed64 parameter);
        return parameter;
    }

    /// <summary>
    /// Compares the exact squared distances between two pairs of points without fixed-point saturation.
    /// </summary>
    /// <returns>A negative value when the left distance is shorter, zero when the
    /// distances are equal, or a positive value when the left distance is longer.</returns>
    /// <param name="leftStart">The start of the left point pair.</param>
    /// <param name="leftEnd">The end of the left point pair.</param>
    /// <param name="rightStart">The start of the right point pair.</param>
    /// <param name="rightEnd">The end of the right point pair.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CompareDistanceSquared(
        Vector2d leftStart,
        Vector2d leftEnd,
        Vector2d rightStart,
        Vector2d rightEnd)
    {
        Signed192 leftDistance = WideGeometry.GetDifferenceDotProduct2D(
            leftStart.X, leftEnd.X, leftStart.Y, leftEnd.Y,
            leftStart.X, leftEnd.X, leftStart.Y, leftEnd.Y);
        Signed192 rightDistance = WideGeometry.GetDifferenceDotProduct2D(
            rightStart.X, rightEnd.X, rightStart.Y, rightEnd.Y,
            rightStart.X, rightEnd.X, rightStart.Y, rightEnd.Y);

        return WideArithmetic.CompareMagnitude(leftDistance, rightDistance);
    }

    /// <summary>
    /// Returns the exact orientation sign of the ordered points without fixed-point saturation.
    /// </summary>
    /// <returns><c>1</c> for a counter-clockwise turn, <c>-1</c> for a clockwise
    /// turn, or <c>0</c> when the points are exactly collinear.</returns>
    /// <param name="origin">The common origin of the two directions.</param>
    /// <param name="first">The endpoint of the first direction.</param>
    /// <param name="second">The endpoint of the second direction in the same coordinate space.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int OrientationSign(Vector2d origin, Vector2d first, Vector2d second) =>
        WideGeometry.GetDifferenceCrossProduct2D(
            first.X, origin.X, first.Y, origin.Y,
            second.X, origin.X, second.Y, origin.Y).Sign;

    /// <summary>
    /// Calculates the forward direction vector in 2D based on a yaw (angle).
    /// </summary>
    /// <remarks>
    /// This is a polar-angle helper: angle zero points along <see cref="Vector2d.Right"/>.
    /// It intentionally differs from the named <see cref="Vector2d.Forward"/> plane constant,
    /// which is <c>+Y</c>.
    /// </remarks>
    /// <param name="angle">The angle in radians representing the rotation in 2D space.</param>
    /// <returns>A unit vector representing the forward direction.</returns>
    public static Vector2d ForwardDirection(Fixed64 angle)
    {
        Fixed64 x = FixedMath.Cos(angle); // Forward in the x-direction
        Fixed64 y = FixedMath.Sin(angle); // Forward in the y-direction
        return new Vector2d(x, y);
    }

    /// <summary>
    /// Dot Product of two vectors.
    /// </summary>
    /// <param name="lhs">The first vector.</param>
    /// <param name="rhs">The second vector in the same coordinate space.</param>
    /// <returns>The sum of the component products using rounded, saturating fixed-point operations.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(Vector2d lhs, Vector2d rhs) => lhs.Dot(rhs.X, rhs.Y);

    /// <summary>Computes the signed 2D cross product.</summary>
    /// <param name="lhs">The first vector.</param>
    /// <param name="rhs">The second vector.</param>
    /// <returns>The signed determinant. A negative value denotes a clockwise turn and a positive value denotes a counterclockwise turn for representable calculations.</returns>
    /// <remarks>The determinant uses rounded, saturating fixed-point products and subtraction.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 CrossProduct(Vector2d lhs, Vector2d rhs) => lhs.CrossProduct(rhs);

    /// <summary>
    /// Rotates this vector by the specified angle (in radians).
    /// </summary>
    /// <param name="vec">The vector to rotate.</param>
    /// <param name="angleInRadians">The angle in radians.</param>
    /// <returns>The rotated vector.</returns>
    public static Vector2d Rotate(Vector2d vec, Fixed64 angleInRadians)
    {
        Fixed64 cos = FixedMath.Cos(angleInRadians);
        Fixed64 sin = FixedMath.Sin(angleInRadians);
        return new Vector2d(
            vec.X * cos - vec.Y * sin,
            vec.X * sin + vec.Y * cos
        );
    }

    /// <summary>
    /// Attempts to rotate a vector by the specified angle with one final
    /// round-half-to-even conversion per component.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> when both final components are representable;
    /// otherwise, <see langword="false"/> and <paramref name="result"/> is
    /// <see langword="default"/>.
    /// </returns>
    /// <param name="vector">The input vector.</param>
    /// <param name="angleInRadians">The rotation in radians, positive from +X toward +Y.</param>
    /// <param name="result">The rotated vector on success, or the zero vector on failure.</param>
    public static bool TryRotate(
        Vector2d vector,
        Fixed64 angleInRadians,
        out Vector2d result)
    {
        Fixed64 cosine = FixedMath.Cos(angleInRadians);
        Fixed64 sine = FixedMath.Sin(angleInRadians);
        bool representable = Fixed64.TrySubtractProducts(
                vector.X,
                cosine,
                vector.Y,
                sine,
                out Fixed64 x)
            & Fixed64.TryAddProducts(
                vector.X,
                sine,
                vector.Y,
                cosine,
                out Fixed64 y);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    /// <summary>
    /// Attempts to transform a local point by a rotation and origin with one
    /// final round-half-to-even conversion per component.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> when both final world components are
    /// representable; otherwise, <see langword="false"/> and
    /// <paramref name="result"/> is <see langword="default"/>.
    /// </returns>
    /// <param name="origin">The origin of the local frame, expressed in world coordinates.</param>
    /// <param name="localPoint">The point in local coordinates.</param>
    /// <param name="angleInRadians">The local-to-world rotation in radians, positive from +X toward +Y.</param>
    /// <param name="result">The rounded point in world coordinates on success, or the zero vector on failure.</param>
    public static bool TryTransformPoint(
        Vector2d origin,
        Vector2d localPoint,
        Fixed64 angleInRadians,
        out Vector2d result)
    {
        if (angleInRadians == Fixed64.Zero)
            return TryAdd(origin, localPoint, out result);

        return WideVector2dTransform.TryTransformPoint(
            origin,
            localPoint,
            angleInRadians,
            out result);
    }

    /// <summary>
    /// Attempts to transform a local point by a rotation and add two origins
    /// with one final round-half-to-even conversion per component.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> when both final world components are
    /// representable; otherwise, <see langword="false"/> and
    /// <paramref name="result"/> is <see langword="default"/>.
    /// </returns>
    /// <param name="firstOrigin">The first translation expressed in world coordinates.</param>
    /// <param name="secondOrigin">The second translation expressed in world coordinates.</param>
    /// <param name="localPoint">The point in local coordinates.</param>
    /// <param name="angleInRadians">The local-to-world rotation in radians, positive from +X toward +Y.</param>
    /// <param name="result">The rounded point in world coordinates on success, or the zero vector on failure.</param>
    public static bool TryTransformPoint(
        Vector2d firstOrigin,
        Vector2d secondOrigin,
        Vector2d localPoint,
        Fixed64 angleInRadians,
        out Vector2d result) =>
        WideVector2dTransform.TryTransformPoint(
            firstOrigin,
            secondOrigin,
            localPoint,
            angleInRadians,
            out result);

    /// <summary>
    /// Attempts to obtain the exact relative offset
    /// <c>firstOrigin + firstOffset - secondOrigin - Rotate(secondLocalPoint)</c>.
    /// </summary>
    /// <remarks>
    /// No rotated point or intermediate sum is narrowed independently.
    /// </remarks>
    /// <returns>
    /// <see langword="true"/> when both final components are representable;
    /// otherwise, <see langword="false"/> and <paramref name="result"/> is
    /// <see langword="default"/>.
    /// </returns>
    /// <param name="firstOrigin">The first frame origin in world coordinates.</param>
    /// <param name="secondOrigin">The second frame origin in world coordinates.</param>
    /// <param name="secondLocalPoint">The point expressed relative to the second frame.</param>
    /// <param name="result">The difference between the two world points on success, or the zero vector on failure.</param>
    /// <param name="firstOffset">The first point offset in world axes.</param>
    /// <param name="angleInRadians">The local-to-world rotation in radians, positive from +X toward +Y.</param>
    public static bool TryGetRelativeOffset(
        Vector2d firstOrigin,
        Vector2d firstOffset,
        Vector2d secondOrigin,
        Vector2d secondLocalPoint,
        Fixed64 angleInRadians,
        out Vector2d result)
    {
        if (angleInRadians == Fixed64.Zero)
        {
            return TrySubtractSums(
                firstOrigin,
                firstOffset,
                secondOrigin,
                secondLocalPoint,
                out result);
        }

        return WideVector2dTransform.TryGetRelativeOffset(
            firstOrigin,
            firstOffset,
            secondOrigin,
            secondLocalPoint,
            angleInRadians,
            out result);
    }

    /// <summary>
    /// Attempts to obtain the exact relative offset between two transformed
    /// local points.
    /// </summary>
    /// <remarks>
    /// The computed relation is
    /// <c>firstOrigin + Rotate(firstLocalPoint, firstAngleInRadians)
    /// - secondOrigin - Rotate(secondLocalPoint, secondAngleInRadians)</c>.
    /// Neither rotated point nor any intermediate sum is narrowed
    /// independently.
    /// </remarks>
    /// <returns>
    /// <see langword="true"/> when both final components are representable;
    /// otherwise, <see langword="false"/> and <paramref name="result"/> is
    /// <see langword="default"/>.
    /// </returns>
    /// <param name="firstOrigin">The first frame origin in world coordinates.</param>
    /// <param name="secondOrigin">The second frame origin in world coordinates.</param>
    /// <param name="secondLocalPoint">The point expressed relative to the second frame.</param>
    /// <param name="result">The difference between the two world points on success, or the zero vector on failure.</param>
    /// <param name="firstLocalPoint">The point expressed relative to the first frame.</param>
    /// <param name="firstAngleInRadians">The first local-to-world rotation in radians.</param>
    /// <param name="secondAngleInRadians">The second local-to-world rotation in radians.</param>
    public static bool TryGetRelativeOffset(
        Vector2d firstOrigin,
        Vector2d firstLocalPoint,
        Fixed64 firstAngleInRadians,
        Vector2d secondOrigin,
        Vector2d secondLocalPoint,
        Fixed64 secondAngleInRadians,
        out Vector2d result)
    {
        if (firstAngleInRadians == Fixed64.Zero
            && secondAngleInRadians == Fixed64.Zero)
        {
            return TrySubtractSums(
                firstOrigin,
                firstLocalPoint,
                secondOrigin,
                secondLocalPoint,
                out result);
        }

        return WideVector2dTransform.TryGetRelativeOffset(
            firstOrigin,
            firstLocalPoint,
            firstAngleInRadians,
            secondOrigin,
            secondLocalPoint,
            secondAngleInRadians,
            out result);
    }
}
