//=======================================================================
// Vector2d.Statics.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using FixedMathSharp.Bounds;

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
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Multiply(Vector2d v1, Vector2d v2) => v1 * v2;

    /// <summary>
    /// Multiplies each vector component by the specified scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Multiply(Vector2d value, Fixed64 factor) => value * factor;

    /// <summary>
    /// Divides each component of the first vector by the corresponding component of the second vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Divide(Vector2d v1, Vector2d v2) => new(v1.X / v2.X, v1.Y / v2.Y);

    /// <summary>
    /// Divides each vector component by the specified scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Divide(Vector2d value, Fixed64 divisor) => value / divisor;

    /// <summary>
    /// Clamps each component of the given vector within the specified min and max bounds.
    /// </summary>
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
            return WideGeometry.GetNormalized(value);

        if (mag <= FixedMath.ScaleSafeMagnitudeThreshold)
            return GetScaleNormalized(value);

        var normalized = new Vector2d(
            FixedMath.FastDiv(value.X, mag),
            FixedMath.FastDiv(value.Y, mag));
        return normalized.IsNormalized()
            ? normalized
            : WideGeometry.GetNormalized(value);
    }

    internal static Vector2d GetScaleNormalized(Vector2d value)
    {
        Fixed64 scale = FixedMath.Max(value.X.Abs(), value.Y.Abs());
        Vector2d scaled = value / scale;
        Fixed64 scaledMagnitude = FixedMath.GetScaledMagnitude(
            scaled.X,
            scaled.Y,
            Fixed64.Zero,
            Fixed64.Zero);
        return scaled / scaledMagnitude;
    }

    /// <summary>
    /// Attempts to transform a component-scaled local point by a rotation and
    /// world origin with one final round-half-to-even conversion per
    /// component.
    /// </summary>
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
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d GetDirection(Vector2d start, Vector2d end) =>
        WideGeometry.GetDirection(start, end);

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
    /// <remarks>
    /// Zero-weight values are ignored. The operation returns
    /// <see langword="false"/> only when the total weight is zero.
    /// </remarks>
    public static bool TryGetWeightedAverage(
        ReadOnlySpan<Vector2d> values,
        ReadOnlySpan<Fixed64> weights,
        out Vector2d average)
    {
        WideWeightedAverage.ValidateInputs(values.Length, weights);
        return WideWeightedAverage.TryGet(values, weights, out average);
    }

    /// <summary>
    /// Creates a vector from a given angle in radians.
    /// </summary>
    public static Vector2d CreateRotation(Fixed64 angle) => new(FixedMath.Cos(angle), FixedMath.Sin(angle));

    /// <summary>
    /// Linearly interpolates between two vectors.
    /// </summary>
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

    /// <summary>
    /// Computes the distance between two vectors using the Euclidean distance formula.
    /// </summary>
    /// <param name="start">The starting vector.</param>
    /// <param name="end">The ending vector.</param>
    /// <returns>The Euclidean distance between the two vectors.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Distance(Vector2d start, Vector2d end) => start.Distance(end);

    /// <summary>
    /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
    /// </summary>
    /// <returns>The squared distance between the two vectors.</returns>
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
    /// <param name="lhs"></param>
    /// <param name="rhs"></param>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(Vector2d lhs, Vector2d rhs) => lhs.Dot(rhs.X, rhs.Y);

    /// <summary>
    /// Cross Product of two vectors.
    /// </summary>
    /// <param name="lhs"></param>
    /// <param name="rhs"></param>
    /// <returns></returns>
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
