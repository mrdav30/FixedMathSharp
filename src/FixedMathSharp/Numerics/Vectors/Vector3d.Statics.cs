//=======================================================================
// Vector3d.Statics.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Vector3d
{
    #region Static Operations

    /// <summary>
    /// Adds two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Add(Vector3d v1, Vector3d v2) => v1 + v2;

    /// <summary>
    /// Subtracts two vectors component-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Subtract(Vector3d v1, Vector3d v2) => v1 - v2;

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
        Fixed64 mag = GetMagnitude(value);

        // If magnitude is zero, return a zero vector to avoid divide-by-zero errors
        if (mag == Fixed64.Zero)
            return new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        // If already normalized, return as-is
        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return value;

        // Normalize it exactly
        return new Vector3d(
            Fixed64.DivideByPositive(value.X, mag),
            Fixed64.DivideByPositive(value.Y, mag),
            Fixed64.DivideByPositive(value.Z, mag)
        );
    }

    /// <summary>
    /// Returns the magnitude (length) of this vector.
    /// </summary>
    /// <param name="vector">The vector whose magnitude is being calculated.</param>
    /// <returns>The magnitude of the vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 GetMagnitude(Vector3d vector)
    {
        Fixed64 mag = (vector.X * vector.X) + (vector.Y * vector.Y) + (vector.Z * vector.Z);

        // Clamp tiny drift around 1 in either direction.
        if (FixedMath.Abs(mag - Fixed64.One) <= Fixed64.Epsilon)
            return Fixed64.One;

        return mag != Fixed64.Zero ? FixedMath.Sqrt(mag) : Fixed64.Zero;
    }

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
        new((v1.X + v2.X) * Fixed64.Half, (v1.Y + v2.Y) * Fixed64.Half, (v1.Z + v2.Z) * Fixed64.Half);

    /// <inheritdoc cref="Distance(Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Distance(Vector3d start, Vector3d end) => start.Distance(end.X, end.Y, end.Z);

    /// <inheritdoc cref="DistanceSquared(Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 DistanceSquared(Vector3d start, Vector3d end) => start.DistanceSquared(end.X, end.Y, end.Z);

    /// <summary>
    /// Calculates the closest points on two line segments.
    /// </summary>
    /// <param name="line1Start">The starting point of the first line segment.</param>
    /// <param name="line1End">The ending point of the first line segment.</param>
    /// <param name="line2Start">The starting point of the second line segment.</param>
    /// <param name="line2End">The ending point of the second line segment.</param>
    /// <returns>
    /// A tuple containing two points representing the closest points on each line segment. 
    /// The first item is the closest point on the first line,
    /// and the second item is the closest point on the second line.
    /// </returns>
    /// <remarks>
    /// This method considers the line segments, not the infinite lines they represent, 
    /// ensuring that the returned points always lie within the provided segments.
    /// </remarks>
    public static (Vector3d, Vector3d) ClosestPointsOnTwoLines(
        Vector3d line1Start,
        Vector3d line1End,
        Vector3d line2Start,
        Vector3d line2End)
    {
        Vector3d u = line1End - line1Start;
        Vector3d v = line2End - line2Start;
        Vector3d w = line1Start - line2Start;

        Fixed64 a = Dot(u, u);
        Fixed64 b = Dot(u, v);
        Fixed64 c = Dot(v, v);
        Fixed64 d = Dot(u, w);
        Fixed64 e = Dot(v, w);
        Fixed64 D = a * c - b * b;

        (Fixed64 sc, Fixed64 tc) = SolveClosestLineParameters(a, b, c, d, e, D);

        // recompute sc if it is outside [0,1]
        if (sc < Fixed64.Zero)
        {
            sc = Fixed64.Zero;
            tc = ClampSegmentParameter(e, c);
        }
        else if (sc > Fixed64.One)
        {
            sc = Fixed64.One;
            tc = ClampSegmentParameter(e + b, c);
        }

        // recompute tc if it is outside [0,1]
        if (tc < Fixed64.Zero)
        {
            tc = Fixed64.Zero;
            sc = ClampSegmentParameter(-d, a);
        }
        else if (tc > Fixed64.One)
        {
            tc = Fixed64.One;
            sc = ClampSegmentParameter(-d + b, a);
        }

        // get the difference of the two closest points
        Vector3d pointOnLine1 = line1Start + sc * u;
        Vector3d pointOnLine2 = line2Start + tc * v;

        return (pointOnLine1, pointOnLine2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (Fixed64 sc, Fixed64 tc) SolveClosestLineParameters(
        Fixed64 a,
        Fixed64 b,
        Fixed64 c,
        Fixed64 d,
        Fixed64 e,
        Fixed64 determinant)
    {
        if (determinant.Abs() < Fixed64.Epsilon)
            return (Fixed64.Zero, b > c ? d / b : e / c);

        return ((b * e - c * d) / determinant, (a * e - b * d) / determinant);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ClampSegmentParameter(Fixed64 numerator, Fixed64 denominator)
    {
        if (numerator < Fixed64.Zero)
            return Fixed64.Zero;

        if (numerator > denominator)
            return Fixed64.One;

        return numerator / denominator;
    }

    /// <summary>
    /// Calculates the closest point on a line segment defined by start and end points to a given point in space.
    /// </summary>
    /// <param name="point">The point to project onto the segment.</param>
    /// <param name="start">The start of the line segment.</param>
    /// <param name="end">The end of the line segment.</param>
    /// <returns>The closest point on the line segment to the given point.</returns>
    public static Vector3d ClosestPointOnLineSegment(Vector3d point, Vector3d start, Vector3d end)
    {
        Vector3d segment = end - start;
        Fixed64 lengthSquared = segment.MagnitudeSquared;

        if (lengthSquared == Fixed64.Zero)
            return start;

        Fixed64 t = Dot(point - start, segment) / lengthSquared;
        t = FixedMath.Clamp(t, Fixed64.Zero, Fixed64.One);

        return start + segment * t;
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
        return point - plane.Normal * Fixed64.DivideByPositive(distance, normalLengthSquared);
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
    /// <param name="value1">The first vertex of the triangle.</param>
    /// <param name="value2">The second vertex of the triangle.</param>
    /// <param name="value3">The third vertex of the triangle.</param>
    /// <param name="amount1">The first barycentric scalar which represents the weighting factor for the second vertex.</param>
    /// <param name="amount2">The second barycentric scalar which represents the weighting factor for the third vertex.</param>
    /// <returns>The cartesian translation represented by the barycentric coordinates within the triangle.</returns>
    public static Vector3d BarycentricCoordinates(
        Vector3d value1,
        Vector3d value2,
        Vector3d value3,
        Fixed64 amount1,
        Fixed64 amount2)
    {
        return new(
            FixedMath.BarycentricCoordinate(value1.X, value2.X, value3.X, amount1, amount2),
            FixedMath.BarycentricCoordinate(value1.Y, value2.Y, value3.Y, amount1, amount2),
            FixedMath.BarycentricCoordinate(value1.Z, value2.Z, value3.Z, amount1, amount2));
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
