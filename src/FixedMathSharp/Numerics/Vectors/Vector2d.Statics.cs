//=======================================================================
// Vector2d.Statics.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Vector2d
{
    #region Static Operations


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
        bool magnitudeIsRepresentable = TryGetMagnitude(value, out Fixed64 mag);

        if (mag == Fixed64.Zero)
            return new Vector2d(Fixed64.Zero, Fixed64.Zero);

        if (!magnitudeIsRepresentable)
            return GetNormalized(value * Fixed64.Half);

        if (mag <= FixedMath.ScaleSafeMagnitudeThreshold)
            return GetScaleNormalized(value);

        // If already normalized, return as-is
        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return value;

        // Normalize it exactly
        return new Vector2d(
            FixedMath.FastDiv(value.X, mag),
            FixedMath.FastDiv(value.Y, mag)
        );
    }

    private static Vector2d GetScaleNormalized(Vector2d value)
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
    public static bool TryGetMagnitude(Vector2d vector, out Fixed64 magnitude)
    {
        Fixed64 mag = (vector.X * vector.X) + (vector.Y * vector.Y);

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

        // If rounding error pushed magnitude slightly above 1, clamp it
        if (mag > Fixed64.One && mag <= Fixed64.One + Fixed64.Epsilon)
        {
            magnitude = Fixed64.One;
            return true;
        }

        magnitude = FixedMath.Sqrt(mag);
        return true;
    }

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
        return new Vector2d(a.X + (b.X - a.X) * amount, a.Y + (b.Y - a.Y) * amount);
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

    internal static int CompareDistanceSquared(
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

    #endregion
}
