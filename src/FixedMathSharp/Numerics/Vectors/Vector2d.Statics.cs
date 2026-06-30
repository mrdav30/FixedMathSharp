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
    /// Subtracts two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Subtract(Vector2d v1, Vector2d v2) => v1 - v2;

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
        Fixed64 mag = GetMagnitude(value);

        if (mag == Fixed64.Zero)
            return new Vector2d(Fixed64.Zero, Fixed64.Zero);

        // If already normalized, return as-is
        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return value;

        // Normalize it exactly
        return new Vector2d(
            FixedMath.FastDiv(value.X, mag),
            FixedMath.FastDiv(value.Y, mag)
        );
    }

    /// <summary>
    /// Returns the magnitude (length) of the given vector.
    /// </summary>
    /// <param name="vector">The vector to compute the magnitude of.</param>
    /// <returns>The magnitude (length) of the vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 GetMagnitude(Vector2d vector)
    {
        Fixed64 mag = (vector.X * vector.X) + (vector.Y * vector.Y);

        // If rounding error pushed magnitude slightly above 1, clamp it
        if (mag > Fixed64.One && mag <= Fixed64.One + Fixed64.Epsilon)
            return Fixed64.One;

        return mag.Abs() > Fixed64.Zero ? FixedMath.Sqrt(mag) : Fixed64.Zero;
    }

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
        Vector2d segment = end - start;
        Fixed64 lengthSquared = segment.MagnitudeSquared;

        if (lengthSquared == Fixed64.Zero)
            return start;

        Fixed64 t = Dot(point - start, segment) / lengthSquared;
        t = FixedMath.Clamp(t, Fixed64.Zero, Fixed64.One);

        return start + (segment * t);
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
