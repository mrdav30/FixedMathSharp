//=======================================================================
// Vector3d.Statics.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Static factory methods and arithmetic operations for <see cref="Vector3d"/>.
/// </content>
public partial struct Vector3d
{
    #region Static Operations

    /// <summary>
    /// Adds two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Add(Vector3d v1, Vector3d v2) => v1 + v2;

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
    public static bool TryAdd(Vector3d left, Vector3d right, out Vector3d result)
    {
        if (!Fixed64.TryAdd(left.X, right.X, out Fixed64 x)
            || !Fixed64.TryAdd(left.Y, right.Y, out Fixed64 y)
            || !Fixed64.TryAdd(left.Z, right.Z, out Fixed64 z))
        {
            result = default;
            return false;
        }

        result = new Vector3d(x, y, z);
        return true;
    }

    /// <summary>
    /// Subtracts two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Subtract(Vector3d v1, Vector3d v2) => v1 - v2;

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
    public static bool TrySubtract(Vector3d left, Vector3d right, out Vector3d result)
    {
        if (!Fixed64.TrySubtract(left.X, right.X, out Fixed64 x)
            || !Fixed64.TrySubtract(left.Y, right.Y, out Fixed64 y)
            || !Fixed64.TrySubtract(left.Z, right.Z, out Fixed64 z))
        {
            result = default;
            return false;
        }

        result = new Vector3d(x, y, z);
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
        Vector3d firstAddend,
        Vector3d secondAddend,
        Vector3d subtrahend,
        out Vector3d result)
    {
        if (!Fixed64.TryAddSubtract(firstAddend.X, secondAddend.X, subtrahend.X, out Fixed64 x)
            || !Fixed64.TryAddSubtract(firstAddend.Y, secondAddend.Y, subtrahend.Y, out Fixed64 y)
            || !Fixed64.TryAddSubtract(firstAddend.Z, secondAddend.Z, subtrahend.Z, out Fixed64 z))
        {
            result = default;
            return false;
        }

        result = new Vector3d(x, y, z);
        return true;
    }

    /// <summary>
    /// Attempts to compute <c>(firstLeft + firstRight) -
    /// (secondLeft + secondRight)</c> component-wise without intermediate
    /// saturation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TrySubtractSums(
        Vector3d firstLeft,
        Vector3d firstRight,
        Vector3d secondLeft,
        Vector3d secondRight,
        out Vector3d result)
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
                out Fixed64 y)
            & Fixed64.TrySubtractSums(
                firstLeft.Z,
                firstRight.Z,
                secondLeft.Z,
                secondRight.Z,
                out Fixed64 z);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector3d(x, y, z);
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
    public static int CompareProjection(Vector3d candidate, Vector3d current, Vector3d direction) =>
        WideGeometry.CompareDifferenceProjection(
            candidate.X,
            current.X,
            direction.X,
            candidate.Y,
            current.Y,
            direction.Y,
            candidate.Z,
            current.Z,
            direction.Z);

    /// <summary>
    /// Projects <paramref name="target"/> minus <paramref name="source"/> onto a direction
    /// without intermediate saturation, then returns a conservative nonnegative result.
    /// </summary>
    /// <param name="target">The target point.</param>
    /// <param name="source">The source point.</param>
    /// <param name="direction">The projection direction; it need not be normalized.</param>
    /// <returns>
    /// Zero for a nonpositive projection, the positive projection floored to Q32.32, or
    /// <see cref="Fixed64.MaxValue"/> when only the final result is unrepresentable.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 ProjectNonNegativeDifference(
        Vector3d target,
        Vector3d source,
        Vector3d direction) =>
        Fixed64.ProjectNonNegativeDifference(
            target.X,
            source.X,
            direction.X,
            target.Y,
            source.Y,
            direction.Y,
            target.Z,
            source.Z,
            direction.Z);

    /// <summary>
    /// Projects <paramref name="target"/> minus <paramref name="source"/> onto
    /// <paramref name="direction"/> and returns the nonnegative parametric
    /// coordinate along that direction.
    /// </summary>
    /// <remarks>
    /// Unlike <see cref="ProjectNonNegativeDifference"/>, this method divides
    /// the exact dot product by the direction's exact squared length. The
    /// direction therefore need not have an exactly unit squared length.
    /// Zero and negative projections return zero; an unrepresentable positive
    /// parameter saturates to <see cref="Fixed64.MaxValue"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 ProjectNonNegativeDifferenceParameter(
        Vector3d target,
        Vector3d source,
        Vector3d direction) =>
        WideGeometry.GetNonNegativeDifferenceProjectionParameter(target, source, direction);

    /// <summary>
    /// Multiplies two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Multiply(Vector3d v1, Vector3d v2) => v1 * v2;

    /// <summary>
    /// Multiplies each vector component by the specified scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Multiply(Vector3d value, Fixed64 factor) => value * factor;

    /// <summary>
    /// Divides each component of the first vector by the corresponding component of the second vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Divide(Vector3d v1, Vector3d v2) => v1 / v2;

    /// <summary>
    /// Divides each vector component by the specified scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Divide(Vector3d value, Fixed64 divisor) => value / divisor;

    /// <summary>
    /// Linearly interpolates between two points.
    /// </summary>
    /// <param name="a">Start value, returned when t = 0.</param>
    /// <param name="b">End value, returned when t = 1.</param>
    /// <param name="mag">Value used to interpolate between a and b.</param>
    /// <returns> Interpolated value, equals to a + (b - a) * t.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Lerp(Vector3d a, Vector3d b, Fixed64 mag)
    {
        mag = FixedMath.Clamp01(mag);
        return new Vector3d(
            FixedMath.Lerp(a.X, b.X, mag),
            FixedMath.Lerp(a.Y, b.Y, mag),
            FixedMath.Lerp(a.Z, b.Z, mag));
    }

    /// <summary>
    /// Linearly interpolates between two vectors without clamping the interpolation factor between 0 and 1.
    /// </summary>
    /// <param name="a">The start vector.</param>
    /// <param name="b">The end vector.</param>
    /// <param name="t">The interpolation factor. Values outside the range [0, 1] will cause the interpolation to go beyond the start or end points.</param>
    /// <returns>The interpolated vector.</returns>
    /// <remarks>
    /// Unlike traditional Lerp, this function allows interpolation factors greater than 1 or less than 0, 
    /// which means the resulting vector can extend beyond the endpoints.
    /// </remarks>
    public static Vector3d UnclampedLerp(Vector3d a, Vector3d b, Fixed64 t) => (b - a) * t + a;

    /// <summary>
    /// Moves from a to b at some speed dependent of a delta time with out passing b.
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <param name="speed"></param>
    /// <param name="dt"></param>
    /// <returns></returns>
    public static Vector3d SpeedLerp(Vector3d a, Vector3d b, Fixed64 speed, Fixed64 dt)
    {
        Vector3d v = b - a;
        Fixed64 dv = speed * dt;
        return dv > v.Magnitude
            ? b
            : a + v.Normalized * dv;
    }

    /// <summary>
    /// Spherically interpolates between two vectors, moving along the shortest arc on a unit sphere.
    /// </summary>
    /// <param name="start">The starting vector.</param>
    /// <param name="end">The ending vector.</param>
    /// <param name="percent">A value between 0 and 1 that represents the interpolation amount. 0 returns the start vector, and 1 returns the end vector.</param>
    /// <returns>The interpolated vector between the two input vectors.</returns>
    /// <remarks>
    /// Slerp is used to interpolate between two unit vectors on a sphere, providing smooth rotation.
    /// It can be more computationally expensive than linear interpolation (Lerp) but results in smoother, arc-like motion.
    /// </remarks>
    public static Vector3d Slerp(Vector3d start, Vector3d end, Fixed64 percent)
    {
        // Dot product - the cosine of the angle between 2 vectors.
        Fixed64 dot = Dot(start, end);
        // Clamp it to be in the range of Acos()
        // This may be unnecessary, but floating point
        // precision can be a fickle mistress.
        dot = FixedMath.Clamp(dot, -Fixed64.One, Fixed64.One);
        // Acos(dot) returns the angle between start and end,
        // And multiplying that by percent returns the angle between
        // start and the final result.
        Fixed64 theta = FixedMath.Acos(dot) * percent;
        Vector3d RelativeVec = end - start * dot;
        RelativeVec.NormalizeInPlace();
        // Orthonormal basis
        // The final result.
        return (start * FixedMath.Cos(theta)) + (RelativeVec * FixedMath.Sin(theta));
    }

    /// <summary>
    /// Calculates a position between four points using Catmull-Rom interpolation.
    /// </summary>
    /// <param name="value1">The first point.</param>
    /// <param name="value2">The second point.</param>
    /// <param name="value3">The third point.</param>
    /// <param name="value4">The fourth point.</param>
    /// <param name="amount">The interpolation factor.</param>
    /// <returns>The interpolated position.</returns>
    public static Vector3d CatmullRom(
        Vector3d value1,
        Vector3d value2,
        Vector3d value3,
        Vector3d value4,
        Fixed64 amount)
    {
        return new Vector3d(
            FixedMath.CatmullRom(value1.X, value2.X, value3.X, value4.X, amount),
            FixedMath.CatmullRom(value1.Y, value2.Y, value3.Y, value4.Y, amount),
            FixedMath.CatmullRom(value1.Z, value2.Z, value3.Z, value4.Z, amount)
        );
    }

    /// <summary>
    /// Calculates a position between two points using Hermite spline interpolation, 
    /// which takes into account the tangents at the endpoints for smoother transitions.
    /// </summary>
    /// <param name="value1">The first point.</param>
    /// <param name="tangent1">The tangent at the first point.</param>
    /// <param name="value2">The second point.</param>
    /// <param name="tangent2">The tangent at the second point.</param>
    /// <param name="amount">The interpolation factor.</param>
    /// <returns>The interpolated position.</returns>
    public static Vector3d HermiteSpline(
        Vector3d value1,
        Vector3d tangent1,
        Vector3d value2,
        Vector3d tangent2,
        Fixed64 amount)
    {
        return new Vector3d(
            FixedMath.HermiteSpline(value1.X, tangent1.X, value2.X, tangent2.X, amount),
            FixedMath.HermiteSpline(value1.Y, tangent1.Y, value2.Y, tangent2.Y, amount),
            FixedMath.HermiteSpline(value1.Z, tangent1.Z, value2.Z, tangent2.Z, amount));
    }

    /// <summary>
    /// Calculates a position between two points using a cubic Hermite interpolation, 
    /// which is similar to HermiteSpline but assumes zero tangents at the endpoints for a smoother curve.
    /// </summary>
    /// <param name="value1">The first point.</param>
    /// <param name="value2">The second point.</param>
    /// <param name="amount">The interpolation factor.</param>
    /// <returns>The interpolated position.</returns>
    public static Vector3d SmoothStep(Vector3d value1, Vector3d value2, Fixed64 amount)
    {
        return new Vector3d(
            FixedMath.SmoothStep(value1.X, value2.X, amount),
            FixedMath.SmoothStep(value1.Y, value2.Y, amount),
            FixedMath.SmoothStep(value1.Z, value2.Z, amount)
        );
    }

    /// <summary>
    /// Normalizes the given vector, returning a unit vector with the same direction.
    /// </summary>
    /// <param name="value">The vector to normalize.</param>
    /// <returns>A normalized (unit) vector with the same direction.</returns>
    public static Vector3d GetNormalized(Vector3d value)
    {
        bool magnitudeIsRepresentable = TryGetMagnitude(
            value,
            out Fixed64 mag,
            out bool isNormalized);

        // If magnitude is zero, return a zero vector to avoid divide-by-zero errors
        if (mag == Fixed64.Zero)
            return new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        if (isNormalized)
            return value;

        if (!magnitudeIsRepresentable || mag == Fixed64.One)
            return WideGeometry.GetNormalized(value);

        if (mag <= FixedMath.ScaleSafeMagnitudeThreshold)
            return GetScaleNormalized(value);

        var normalized = new Vector3d(
            FixedMath.FastDiv(value.X, mag),
            FixedMath.FastDiv(value.Y, mag),
            FixedMath.FastDiv(value.Z, mag));
        return normalized.IsNormalized()
            ? normalized
            : WideGeometry.GetNormalized(value);
    }

    internal static Vector3d GetScaleNormalized(Vector3d value)
    {
        Fixed64 scale = FixedMath.Max(value.X.Abs(), FixedMath.Max(value.Y.Abs(), value.Z.Abs()));
        Vector3d scaled = value / scale;
        Fixed64 scaledMagnitude = FixedMath.GetScaledMagnitude(
            scaled.X,
            scaled.Y,
            scaled.Z,
            Fixed64.Zero);
        return scaled / scaledMagnitude;
    }

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
        Vector3d outerLocalPoint,
        Vector3d outerScale,
        Vector3d innerFrameOffset,
        Vector3d innerFrameScale,
        Vector3d innerLocalDisplacement,
        FixedQuaternion innerRotation,
        out Vector3d result) =>
        WideOrientedBox.TryComposeScaledLocalPoints(
            outerLocalPoint,
            outerScale,
            innerFrameOffset,
            innerFrameScale,
            innerLocalDisplacement,
            innerRotation,
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
    public static Vector3d GetDirection(Vector3d start, Vector3d end) =>
        WideGeometry.GetDirection(start, end);

    /// <summary>
    /// Returns the magnitude (length) of this vector.
    /// </summary>
    /// <param name="vector">The vector whose magnitude is being calculated.</param>
    /// <returns>The magnitude of the vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 GetMagnitude(Vector3d vector)
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
    public static bool TryGetMagnitude(Vector3d vector, out Fixed64 magnitude) =>
        TryGetMagnitude(vector, out magnitude, out _);

    private static bool TryGetMagnitude(
        Vector3d vector,
        out Fixed64 magnitude,
        out bool isNormalized)
    {
        Fixed64 mag = (vector.X * vector.X) + (vector.Y * vector.Y) + (vector.Z * vector.Z);
        isNormalized = mag != Fixed64.Zero
            && FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon;

        if (mag == Fixed64.MaxValue)
            return FixedMath.TryGetScaledMagnitude(
                vector.X,
                vector.Y,
                vector.Z,
                Fixed64.Zero,
                out magnitude);

        if (mag <= FixedMath.ScaleSafeMagnitudeSquaredThreshold)
        {
            magnitude = FixedMath.GetScaledMagnitude(
                vector.X,
                vector.Y,
                vector.Z,
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
    /// any component difference.
    /// </summary>
    /// <param name="start">The first endpoint.</param>
    /// <param name="end">The second endpoint.</param>
    /// <param name="distance">The rounded distance, or <see cref="Fixed64.MaxValue"/> when it is not representable.</param>
    /// <returns><see langword="true"/> when the distance fits in <see cref="Fixed64"/>; otherwise, <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryGetDistance(Vector3d start, Vector3d end, out Fixed64 distance) =>
        WideGeometry.TryGetDistance(start, end, out distance);

    /// <summary>
    /// Compares the exact squared magnitudes of two vectors without fixed-point saturation.
    /// </summary>
    /// <returns>A negative value when <paramref name="left"/> is shorter, zero when the
    /// magnitudes are equal, or a positive value when <paramref name="left"/> is longer.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CompareMagnitudeSquared(Vector3d left, Vector3d right) =>
        Fixed64.CompareMagnitudeSquared(
            left.X,
            left.Y,
            left.Z,
            Fixed64.Zero,
            right.X,
            right.Y,
            right.Z,
            Fixed64.Zero);

    /// <summary>
    /// Compares the exact squared distances between two pairs of points without fixed-point saturation.
    /// </summary>
    /// <returns>A negative value when the left distance is shorter, zero when the
    /// distances are equal, or a positive value when the left distance is longer.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CompareDistanceSquared(
        Vector3d leftStart,
        Vector3d leftEnd,
        Vector3d rightStart,
        Vector3d rightEnd) =>
        WideGeometry.CompareSquaredDistance3D(
            leftStart.X, leftEnd.X,
            leftStart.Y, leftEnd.Y,
            leftStart.Z, leftEnd.Z,
            rightStart.X, rightEnd.X,
            rightStart.Y, rightEnd.Y,
            rightStart.Z, rightEnd.Z);

    /// <summary>
    /// Returns the exact sign of the scalar triple product without fixed-point saturation.
    /// </summary>
    /// <returns><c>1</c> for a positive product, <c>-1</c> for a negative product,
    /// or <c>0</c> when the vectors are exactly coplanar.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int ScalarTripleProductSign(Vector3d first, Vector3d second, Vector3d third) =>
        WideGeometry.GetTripleProductSign(
            first.X, first.Y, first.Z,
            second.X, second.Y, second.Z,
            third.X, third.Y, third.Z);

    /// <summary>
    /// Returns a new <see cref="Vector3d"/> where each component is the absolute value of the corresponding input component.
    /// </summary>
    /// <param name="value">The input vector.</param>
    /// <returns>A vector with absolute values for each component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Abs(Vector3d value) => new(value.X.Abs(), value.Y.Abs(), value.Z.Abs());

    /// <summary>
    /// Returns a new <see cref="Vector3d"/> where each component is the sign of the corresponding input component.
    /// </summary>
    /// <param name="value">The input vector.</param>
    /// <returns>A vector where each component is -1, 0, or 1 based on the sign of the input.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Sign(Vector3d value) => new(value.X.Sign(), value.Y.Sign(), value.Z.Sign());

    /// <summary>
    /// Attempts to calculate the exact non-negative weighted average with one
    /// final round-half-to-even conversion per component.
    /// </summary>
    /// <remarks>
    /// Zero-weight values are ignored. The operation returns
    /// <see langword="false"/> only when the total weight is zero.
    /// </remarks>
    public static bool TryGetWeightedAverage(
        ReadOnlySpan<Vector3d> values,
        ReadOnlySpan<Fixed64> weights,
        out Vector3d average)
    {
        WideWeightedAverage.ValidateInputs(values.Length, weights);
        return WideWeightedAverage.TryGet(values, weights, out average);
    }

    /// <summary>
    /// Clamps each component of the given <see cref="Vector3d"/> within the specified min and max bounds.
    /// </summary>
    /// <param name="value">The vector to clamp.</param>
    /// <param name="min">The minimum bounds.</param>
    /// <param name="max">The maximum bounds.</param>
    /// <returns>A vector with each component clamped between min and max.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Clamp(Vector3d value, Vector3d min, Vector3d max) =>
        new(FixedMath.Clamp(value.X, min.X, max.X),
            FixedMath.Clamp(value.Y, min.Y, max.Y),
            FixedMath.Clamp(value.Z, min.Z, max.Z));

    /// <summary>
    /// Clamps the given Vector3d within the specified magnitude.
    /// </summary>
    /// <param name="value"></param>
    /// <param name="maxMagnitude"></param>
    /// <returns></returns>
    public static Vector3d ClampMagnitude(Vector3d value, Fixed64 maxMagnitude)
    {
        if (value.MagnitudeSquared > maxMagnitude * maxMagnitude)
            return value.Normalized * maxMagnitude; // Clamp magnitude without changing direction

        return value;
    }

    /// <summary>
    /// Determines if two vectors are exactly parallel by checking if their cross product is zero.
    /// </summary>
    /// <param name="v1">The first vector.</param>
    /// <param name="v2">The second vector.</param>
    /// <returns>True if the vectors are exactly parallel, false otherwise.</returns>
    public static bool AreParallel(Vector3d v1, Vector3d v2) => Cross(v1, v2).MagnitudeSquared == Fixed64.Zero;

    /// <summary>
    /// Determines if two vectors are approximately parallel based on a cosine similarity threshold.
    /// </summary>
    /// <param name="v1">The first normalized vector.</param>
    /// <param name="v2">The second normalized vector.</param>
    /// <param name="cosThreshold">The cosine similarity threshold for near-parallel vectors.</param>
    /// <returns>True if the vectors are nearly parallel, false otherwise.</returns>
    public static bool AreAlmostParallel(Vector3d v1, Vector3d v2, Fixed64 cosThreshold)
    {
        // Assuming v1 and v2 are already normalized
        Fixed64 dot = Dot(v1, v2);

        // Compare dot product directly to the cosine threshold
        return dot >= cosThreshold;
    }

    /// <summary>
    /// Computes the midpoint between two vectors.
    /// </summary>
    /// <param name="v1">The first vector.</param>
    /// <param name="v2">The second vector.</param>
    /// <returns>The midpoint vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Midpoint(Vector3d v1, Vector3d v2) =>
        new(FixedMath.Midpoint(v1.X, v2.X), FixedMath.Midpoint(v1.Y, v2.Y), FixedMath.Midpoint(v1.Z, v2.Z));

    /// <inheritdoc cref="Distance(Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Distance(Vector3d start, Vector3d end) => start.Distance(end.X, end.Y, end.Z);

    /// <inheritdoc cref="DistanceSquared(Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 DistanceSquared(Vector3d start, Vector3d end) => start.DistanceSquared(end.X, end.Y, end.Z);

    /// <summary>
    /// Calculates the closest point on a line segment defined by start and end points to a given point in space.
    /// </summary>
    /// <param name="point">The point to project onto the segment.</param>
    /// <param name="start">The start of the line segment.</param>
    /// <param name="end">The end of the line segment.</param>
    /// <returns>The closest point on the line segment to the given point.</returns>
    /// <remarks>
    /// Endpoint differences and projection products are evaluated across the
    /// complete raw domain before the parameter is clamped and rounded.
    /// A direction whose exact Q64.64 squared-length total is at most 2^31 raw
    /// units rounds to zero in Q32.32 and returns <paramref name="start"/>.
    /// </remarks>
    public static Vector3d ClosestPointOnLineSegment(Vector3d point, Vector3d start, Vector3d end)
    {
        Signed192 denominator = WideGeometry.GetDifferenceDotProduct3D(
            end.X, start.X, end.Y, start.Y, end.Z, start.Z,
            end.X, start.X, end.Y, start.Y, end.Z, start.Z);
        if (WideGeometry.IsSquaredLengthDegenerate(denominator))
            return start;

        Signed192 numerator = WideGeometry.GetDifferenceDotProduct3D(
            point.X, start.X, point.Y, start.Y, point.Z, start.Z,
            end.X, start.X, end.Y, start.Y, end.Z, start.Z);
        if (numerator.Sign <= 0)
            return start;
        if (WideArithmetic.CompareMagnitude(numerator, denominator) >= 0)
            return end;

        _ = Fixed64.TryGetUnitIntervalRatio(numerator, denominator, out Fixed64 parameter);
        return new Vector3d(
            FixedMath.Lerp(start.X, end.X, parameter),
            FixedMath.Lerp(start.Y, end.Y, parameter),
            FixedMath.Lerp(start.Z, end.Z, parameter));
    }

    /// <summary>
    /// Dot Product of two vectors.
    /// </summary>
    /// <param name="lhs"></param>
    /// <param name="rhs"></param>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(Vector3d lhs, Vector3d rhs) => lhs.Dot(rhs.X, rhs.Y, rhs.Z);

    /// <summary>
    /// Cross Product of two vectors.
    /// </summary>
    /// <param name="lhs"></param>
    /// <param name="rhs"></param>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Cross(Vector3d lhs, Vector3d rhs) => lhs.Cross(rhs.X, rhs.Y, rhs.Z);

    /// <inheritdoc cref="CrossProduct(Fixed64, Fixed64, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 CrossProduct(Vector3d lhs, Vector3d rhs) => lhs.CrossProduct(rhs.X, rhs.Y, rhs.Z);

    /// <summary>
    /// Projects a vector onto another vector.
    /// </summary>
    /// <param name="vector"></param>
    /// <param name="onNormal"></param>
    /// <returns></returns>
    public static Vector3d Project(Vector3d vector, Vector3d onNormal)
    {
        Fixed64 sqrMag = Dot(onNormal, onNormal);
        if (sqrMag.Abs() < Fixed64.Epsilon)
            return Zero;
        else
        {
            Fixed64 dot = Dot(vector, onNormal);
            return new Vector3d(onNormal.X * dot / sqrMag,
                onNormal.Y * dot / sqrMag,
                onNormal.Z * dot / sqrMag);
        }
    }

    /// <summary>
    /// Projects a vector onto a plane defined by a normal orthogonal to the plane.
    /// </summary>
    /// <param name="vector"></param>
    /// <param name="planeNormal"></param>
    /// <returns></returns>
    public static Vector3d ProjectOnPlane(Vector3d vector, Vector3d planeNormal)
    {
        Fixed64 sqrMag = Dot(planeNormal, planeNormal);
        if (sqrMag.Abs() < Fixed64.Epsilon)
            return vector;
        else
        {
            Fixed64 dot = Dot(vector, planeNormal);
            return new Vector3d(vector.X - planeNormal.X * dot / sqrMag,
                vector.Y - planeNormal.Y * dot / sqrMag,
                vector.Z - planeNormal.Z * dot / sqrMag);
        }
    }

    /// <summary>
    /// Returns the normalized direction of a vector projected onto a plane.
    /// </summary>
    /// <remarks>
    /// The rejection is formed exactly with full-domain intermediates before
    /// returning its nearest representable normalized direction. A zero normal
    /// returns the normalized input vector; a zero projection returns
    /// <see cref="Zero"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d GetNormalizedProjectionOnPlane(
        Vector3d vector,
        Vector3d planeNormal) =>
        WideGeometry.GetNormalizedProjectionOnPlane(vector, planeNormal);

    /// <summary>
    /// Projects a point onto a plane defined by a normal and a distance from the origin.
    /// </summary>
    /// <param name="point">The point to project.</param>
    /// <param name="plane">The plane onto which the point is projected.</param>
    /// <returns>The projected point.</returns>
    public static Vector3d ProjectOnPlane(Vector3d point, FixedPlane plane)
    {
        Fixed64 normalLengthSquared = plane.Normal.MagnitudeSquared;
        if (normalLengthSquared == Fixed64.Zero)
            return point;

        Fixed64 distance = plane.DotCoordinate(point);
        return point - plane.Normal * FixedMath.FastDiv(distance, normalLengthSquared);
    }

    /// <summary>
    /// Computes the angle in degrees between two vectors.
    /// </summary>
    /// <param name="from">The starting vector.</param>
    /// <param name="to">The target vector.</param>
    /// <returns>The angle in degrees between the two vectors.</returns>
    /// <remarks>
    /// This method calculates the angle by using the dot product between the vectors and normalizing the result.
    /// The angle is always the smaller angle between the two vectors on a plane.
    /// </remarks>
    public static Fixed64 Angle(Vector3d from, Vector3d to)
    {
        Fixed64 denominator = FixedMath.Sqrt(from.MagnitudeSquared * to.MagnitudeSquared);

        if (denominator.Abs() < Fixed64.Epsilon)
            return Fixed64.Zero;

        Fixed64 dot = FixedMath.Clamp(Dot(from, to) / denominator, -Fixed64.One, Fixed64.One);

        return FixedMath.RadToDeg(FixedMath.Acos(dot));
    }

    /// <summary>
    /// Calculates the barycentric coordinates of a point with respect to a triangle defined by three vertices.
    /// </summary>
    /// <param name="coordA">The first vertex of the triangle.</param>
    /// <param name="coordB">The second vertex of the triangle.</param>
    /// <param name="coordC">The third vertex of the triangle.</param>
    /// <param name="weightB">The barycentric weight for the second vertex.</param>
    /// <param name="weightC">The barycentric weight for the third vertex.</param>
    /// <returns>The cartesian translation represented by the barycentric coordinates within the triangle.</returns>
    public static Vector3d BarycentricCoordinates(
        Vector3d coordA,
        Vector3d coordB,
        Vector3d coordC,
        Fixed64 weightB,
        Fixed64 weightC)
    {
        return new(
            FixedMath.BarycentricCoordinate(coordA.X, coordB.X, coordC.X, weightB, weightC),
            FixedMath.BarycentricCoordinate(coordA.Y, coordB.Y, coordC.Y, weightB, weightC),
            FixedMath.BarycentricCoordinate(coordA.Z, coordB.Z, coordC.Z, weightB, weightC));
    }

    /// <summary>
    ///  Returns a vector whose elements are the maximum of each of the pairs of elements in two specified vectors.
    /// </summary>
    /// <param name="value1">The first vector.</param>
    /// <param name="value2">The second vector.</param>
    /// <returns>The maximized vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Max(Vector3d value1, Vector3d value2) =>
         new(FixedMath.Max(value1.X, value2.X),
             FixedMath.Max(value1.Y, value2.Y),
             FixedMath.Max(value1.Z, value2.Z));

    /// <summary>
    /// Returns a vector whose elements are the minimum of each of the pairs of elements in two specified vectors.
    /// </summary>
    /// <param name="value1">The first vector.</param>
    /// <param name="value2">The second vector.</param>
    /// <returns>The minimized vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Min(Vector3d value1, Vector3d value2) =>
        new(FixedMath.Min(value1.X, value2.X),
            FixedMath.Min(value1.Y, value2.Y),
            FixedMath.Min(value1.Z, value2.Z));

    /// <summary>
    /// Returns a vector that is the negation of the specified vector, effectively reversing its direction.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Negate(Vector3d value) => -value;

    /// <summary>
    /// Rotates the vector around a given position using a specified quaternion rotation.
    /// </summary>
    /// <param name="source">The vector to rotate.</param>
    /// <param name="position">The position around which the vector is rotated.</param>
    /// <param name="rotation">The quaternion representing the rotation.</param>
    /// <returns>The rotated vector.</returns>
    public static Vector3d Rotate(Vector3d source, Vector3d position, FixedQuaternion rotation)
    {
        source -= position; // Translate the vector by the position
        var normalizedRotation = rotation.Normalized;
        return (normalizedRotation * source) + position;
    }

    /// <summary>
    /// Applies the inverse of a specified quaternion rotation to the vector around a given position.
    /// </summary>
    /// <param name="source">The vector to rotate.</param>
    /// <param name="position">The position around which the vector is rotated.</param>
    /// <param name="rotation">The quaternion representing the inverse rotation.</param>
    /// <returns>The rotated vector.</returns>
    public static Vector3d InverseRotate(Vector3d source, Vector3d position, FixedQuaternion rotation)
    {
        source -= position; // Translate the vector by the position
        var normalizedRotation = rotation.Normalized;
        // Undo the rotation
        source = normalizedRotation.Inverse() * source;
        // Add the original position back
        return source + position;
    }

    /// <summary>
    /// Reflects a vector off the plane defined by a normal. 
    /// The result is a vector that points in the direction a perfectly reflected ray would go, 
    /// based on the incoming vector and the normal of the plane it reflects off.
    /// </summary>
    /// <param name="vector">The vector to reflect.</param>
    /// <param name="normal">The normal of the plane to reflect off.</param>
    /// <returns>The reflected vector.</returns>
    public static Vector3d Reflect(Vector3d vector, Vector3d normal)
    {
        Fixed64 dot = Dot(vector, normal);
        return vector - 2 * dot * normal;
    }

    /// <summary>
    /// Transforms a vector by the given 4x4 matrix, applying rotation, scaling, and translation as defined by the matrix.
    /// </summary>
    /// <param name="vector">The vector to transform.</param>
    /// <param name="matrix">The transformation matrix.</param>
    /// <returns>The transformed vector.</returns>
    /// <remarks>
    /// Same as <see cref="operator *(Vector3d, Fixed4x4)"/>.
    /// </remarks>
    public static Vector3d Transform(Vector3d vector, Fixed4x4 matrix) => matrix * vector;

    #endregion
}
