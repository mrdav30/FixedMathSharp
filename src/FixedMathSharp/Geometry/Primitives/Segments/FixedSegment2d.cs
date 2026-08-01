//=======================================================================
// FixedSegment2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Represents a finite line segment in two-dimensional fixed-point space.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct FixedSegment2d : IEquatable<FixedSegment2d>
{
    #region Fields

    /// <summary>
    /// The start point of the segment.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Vector2d Start;

    /// <summary>
    /// The end point of the segment.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Vector2d End;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new segment from start and end points.
    /// </summary>
    [JsonConstructor]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedSegment2d(Vector2d start, Vector2d end)
    {
        Start = start;
        End = end;
    }

    #endregion

    #region Properties

    /// <summary>
    /// The vector from <see cref="Start"/> to <see cref="End"/>.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Delta
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => End - Start;
    }

    /// <summary>
    /// The segment length.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Length
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Delta.Magnitude;
    }

    /// <summary>
    /// The squared segment length.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 LengthSquared
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Delta.MagnitudeSquared;
    }

    /// <summary>
    /// The normalized axis-aligned area that contains this segment.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public FixedBoundArea Bounds
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => FixedBoundArea.FromMinMax(Start, End);
    }

    #endregion

    #region Spatial Queries

    /// <summary>
    /// Finds the closest point on this finite segment to the supplied point.
    /// </summary>
    /// <remarks>
    /// Zero-length segments deterministically return <see cref="Start"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d ClosestPoint(Vector2d point) => Vector2d.ClosestPointOnLineSegment(point, Start, End);

    /// <summary>
    /// Computes the squared distance from the supplied point to this finite segment.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DistanceSquared(Vector2d point)
    {
        return Vector2d.DistanceSquared(point, ClosestPoint(point));
    }

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a capsule.
    /// </summary>
    /// <remarks>
    /// The capsule is described by its finite center-line segment and radius.
    /// Returned parameters are clamped to [0, 1]. A zero-length capsule axis is
    /// treated as a circle.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="radius"/> is negative.
    /// </exception>
    public readonly bool TryGetCapsuleIntersectionInterval(
        FixedSegment2d capsuleAxis,
        Fixed64 radius,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            capsuleAxis,
            radius,
            Fixed64.Zero,
            out entryParameter,
            out exitParameter);

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a radially
    /// expanded capsule.
    /// </summary>
    /// <remarks>
    /// The authored radius and sweep expansion remain separate until the exact
    /// finite-axis solve. Returned parameters are clamped to [0, 1]. A zero-length
    /// capsule axis is treated as a circle.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="radius"/> or <paramref name="radiusExpansion"/>
    /// is negative.
    /// </exception>
    public readonly bool TryGetCapsuleIntersectionInterval(
        FixedSegment2d capsuleAxis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            capsuleAxis,
            radius,
            radiusExpansion,
            out entryParameter,
            out exitParameter,
            out _,
            out _);

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a capsule
    /// and reports exact endpoint containment.
    /// </summary>
    /// <remarks>
    /// <paramref name="startContained"/> is inclusive of the capsule boundary.
    /// <paramref name="endContainedStrict"/> is true only for the mathematical
    /// interior. Both classifications use the wide solve inputs rather than rounded
    /// parameters or reconstructed points. A zero-length capsule axis is treated as
    /// a circle.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="radius"/> or <paramref name="radiusExpansion"/>
    /// is negative.
    /// </exception>
    public readonly bool TryGetCapsuleIntersectionInterval(
        FixedSegment2d capsuleAxis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool startContained,
        out bool endContainedStrict)
    {
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetCapsuleInterval(
            this,
            capsuleAxis,
            radius,
            radiusExpansion,
            out entryParameter,
            out exitParameter,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// centered capsule.
    /// </summary>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="axisLength"/> or
    /// <paramref name="radius"/> is negative.
    /// </exception>
    public readonly bool TryGetCapsuleIntersectionInterval(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            center,
            axisDirection,
            axisLength,
            radius,
            Fixed64.Zero,
            out entryParameter,
            out exitParameter);

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// centered, radially expanded capsule.
    /// </summary>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="axisLength"/>,
    /// <paramref name="radius"/> or <paramref name="radiusExpansion"/> is
    /// negative.
    /// </exception>
    public readonly bool TryGetCapsuleIntersectionInterval(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            center,
            axisDirection,
            axisLength,
            radius,
            radiusExpansion,
            out entryParameter,
            out exitParameter,
            out _,
            out _);

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// centered, radially expanded capsule and reports exact endpoint
    /// containment.
    /// </summary>
    /// <remarks>
    /// The normalized axis defines the conceptual center-line endpoints as
    /// <c>center +/- axisDirection * (axisLength / 2)</c> without constructing or
    /// narrowing either endpoint. <paramref name="startContained"/> includes the
    /// boundary; <paramref name="endContainedStrict"/> excludes it.
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="axisLength"/>,
    /// <paramref name="radius"/> or <paramref name="radiusExpansion"/> is
    /// negative.
    /// </exception>
    public readonly bool TryGetCapsuleIntersectionInterval(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool startContained,
        out bool endContainedStrict)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Capsule axis direction must be normalized.", nameof(axisDirection));
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetCapsuleInterval(
            this,
            center,
            axisDirection,
            axisLength,
            radius,
            radiusExpansion,
            out entryParameter,
            out exitParameter,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Attempts to find the unique intersection point shared by this segment and another segment.
    /// </summary>
    /// <remarks>
    /// Closed endpoint touches and coincident point segments have one unique intersection.
    /// Collinear segments with a positive-length overlap do not.
    /// </remarks>
    public readonly bool TryGetUniqueIntersection(
        FixedSegment2d other,
        out Fixed64 thisParameter)
    {
        return TryGetUniqueIntersection(other, out thisParameter, out _);
    }

    /// <summary>
    /// Returns the closest finite points on this segment and another segment.
    /// </summary>
    /// <remarks>
    /// Exact shared endpoints are returned in this-start, this-end, other-start,
    /// other-end order before parameterized interior intersections. Remaining endpoint
    /// projections use the same order, and exact distance ties keep the first candidate.
    /// </remarks>
    public readonly (Vector2d ThisPoint, Vector2d OtherPoint) GetClosestPoints(FixedSegment2d other)
    {
        if (PointOnSegment(Start, other))
            return (Start, Start);
        if (PointOnSegment(End, other))
            return (End, End);
        if (PointOnSegment(other.Start, this))
            return (other.Start, other.Start);
        if (PointOnSegment(other.End, this))
            return (other.End, other.End);

        if (TryGetUniqueIntersection(other, out Fixed64 thisParameter, out Fixed64 otherParameter))
        {
            return (Interpolate(this, thisParameter), Interpolate(other, otherParameter));
        }

        Vector2d thisPoint = Start;
        Vector2d otherPoint = other.ClosestPoint(Start);

        ConsiderClosestCandidate(End, other.ClosestPoint(End), ref thisPoint, ref otherPoint);
        ConsiderClosestCandidate(
            Vector2d.ClosestPointOnLineSegment(other.Start, Start, End),
            other.Start,
            ref thisPoint,
            ref otherPoint);
        ConsiderClosestCandidate(
            Vector2d.ClosestPointOnLineSegment(other.End, Start, End),
            other.End,
            ref thisPoint,
            ref otherPoint);

        return (thisPoint, otherPoint);
    }

    private readonly bool TryGetUniqueIntersection(
        FixedSegment2d other,
        out Fixed64 thisParameter,
        out Fixed64 otherParameter)
    {
        bool thisIsPoint = Start == End;
        bool otherIsPoint = other.Start == other.End;

        if (thisIsPoint)
        {
            thisParameter = Fixed64.Zero;
            if (!PointOnSegment(Start, other))
            {
                otherParameter = default;
                return false;
            }

            otherParameter = otherIsPoint
                ? Fixed64.Zero
                : Vector2d.GetClosestPointOnLineSegmentParameter(Start, other.Start, other.End);
            return true;
        }

        if (otherIsPoint)
        {
            otherParameter = Fixed64.Zero;
            if (!PointOnSegment(other.Start, this))
            {
                thisParameter = default;
                return false;
            }

            thisParameter = Vector2d.GetClosestPointOnLineSegmentParameter(other.Start, Start, End);
            return true;
        }

        Signed192 determinant = WideGeometry.GetDifferenceCrossProduct2D(
            End.X, Start.X, End.Y, Start.Y,
            other.End.X, other.Start.X, other.End.Y, other.Start.Y);
        if (!determinant.IsZero)
        {
            Signed192 thisNumerator = WideGeometry.GetDifferenceCrossProduct2D(
                other.Start.X, Start.X, other.Start.Y, Start.Y,
                other.End.X, other.Start.X, other.End.Y, other.Start.Y);
            Signed192 otherNumerator = WideGeometry.GetDifferenceCrossProduct2D(
                other.Start.X, Start.X, other.Start.Y, Start.Y,
                End.X, Start.X, End.Y, Start.Y);

            if (Fixed64.TryGetUnitIntervalRatio(thisNumerator, determinant, out thisParameter)
                && Fixed64.TryGetUnitIntervalRatio(otherNumerator, determinant, out otherParameter))
            {
                return true;
            }

            thisParameter = default;
            otherParameter = default;
            return false;
        }

        Signed192 collinearity = WideGeometry.GetDifferenceCrossProduct2D(
            other.Start.X, Start.X, other.Start.Y, Start.Y,
            End.X, Start.X, End.Y, Start.Y);
        if (!collinearity.IsZero)
        {
            thisParameter = default;
            otherParameter = default;
            return false;
        }

        int sharedPointCount = 0;
        Vector2d sharedPoint = default;
        ConsiderSharedEndpoint(Start, this, other, ref sharedPointCount, ref sharedPoint);
        ConsiderSharedEndpoint(End, this, other, ref sharedPointCount, ref sharedPoint);
        ConsiderSharedEndpoint(other.Start, this, other, ref sharedPointCount, ref sharedPoint);
        ConsiderSharedEndpoint(other.End, this, other, ref sharedPointCount, ref sharedPoint);

        if (sharedPointCount == 1)
        {
            thisParameter = Vector2d.GetClosestPointOnLineSegmentParameter(sharedPoint, Start, End);
            otherParameter = Vector2d.GetClosestPointOnLineSegmentParameter(sharedPoint, other.Start, other.End);
            return true;
        }

        thisParameter = default;
        otherParameter = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d Interpolate(FixedSegment2d segment, Fixed64 parameter)
    {
        return new Vector2d(
            FixedMath.Lerp(segment.Start.X, segment.End.X, parameter),
            FixedMath.Lerp(segment.Start.Y, segment.End.Y, parameter));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool PointOnSegment(Vector2d point, FixedSegment2d segment)
    {
        Signed192 cross = WideGeometry.GetDifferenceCrossProduct2D(
            point.X, segment.Start.X, point.Y, segment.Start.Y,
            segment.End.X, segment.Start.X, segment.End.Y, segment.Start.Y);
        if (!cross.IsZero)
            return false;

        long pointX = point.X.m_rawValue;
        long pointY = point.Y.m_rawValue;
        long startX = segment.Start.X.m_rawValue;
        long startY = segment.Start.Y.m_rawValue;
        long endX = segment.End.X.m_rawValue;
        long endY = segment.End.Y.m_rawValue;
        return pointX >= Math.Min(startX, endX)
            && pointX <= Math.Max(startX, endX)
            && pointY >= Math.Min(startY, endY)
            && pointY <= Math.Max(startY, endY);
    }

    private static void ConsiderSharedEndpoint(
        Vector2d candidate,
        FixedSegment2d first,
        FixedSegment2d second,
        ref int sharedPointCount,
        ref Vector2d sharedPoint)
    {
        if (!PointOnSegment(candidate, first)
            || !PointOnSegment(candidate, second)
            || (sharedPointCount != 0 && candidate == sharedPoint))
        {
            return;
        }

        sharedPoint = candidate;
        sharedPointCount++;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ConsiderClosestCandidate(
        Vector2d candidateThisPoint,
        Vector2d candidateOtherPoint,
        ref Vector2d thisPoint,
        ref Vector2d otherPoint)
    {
        if (Vector2d.CompareDistanceSquared(
            candidateThisPoint,
            candidateOtherPoint,
            thisPoint,
            otherPoint) < 0)
        {
            thisPoint = candidateThisPoint;
            otherPoint = candidateOtherPoint;
        }
    }

    #endregion

    #region Deconstruction

    /// <summary>
    /// Deconstructs the segment into start and end points.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Vector2d start, out Vector2d end)
    {
        start = Start;
        end = End;
    }

    #endregion

    #region Operators

    /// <summary>
    /// Determines whether two segments have the same ordered endpoints.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedSegment2d left, FixedSegment2d right) => left.Equals(right);

    /// <summary>
    /// Determines whether two segments have different ordered endpoints.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedSegment2d left, FixedSegment2d right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedSegment2d other)
    {
        return Start == other.Start && End == other.End;
    }

    /// <inheritdoc />
    public override bool Equals(object? obj)
    {
        return obj is FixedSegment2d other && Equals(other);
    }

    /// <inheritdoc />
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + Start.StateHash;
            hash = (hash * 31) + End.StateHash;
            return hash;
        }
    }

    #endregion

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// circle, mapped to the caller-supplied total distance.
    /// </summary>
    public readonly bool TryGetCircleIntersectionDistanceInterval(
        FixedBoundCircle circle,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetCircleIntersectionDistanceInterval(
            circle,
            Fixed64.Zero,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// radially expanded circle and reports exact endpoint containment.
    /// </summary>
    /// <remarks>
    /// The exact segment interval maps to <c>[0, totalDistance]</c> with one
    /// final round-half-to-even conversion. Start containment is inclusive;
    /// end containment is strict and is classified before distance rounding.
    /// </remarks>
    public readonly bool TryGetCircleIntersectionDistanceInterval(
        FixedBoundCircle circle,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetCircleDistanceInterval(
            this,
            circle,
            radiusExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects an
    /// endpoint-authored capsule.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        FixedSegment2d capsuleAxis,
        Fixed64 radius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetCapsuleIntersectionDistanceInterval(
            capsuleAxis,
            radius,
            Fixed64.Zero,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects an
    /// endpoint-authored, radially expanded capsule and reports exact endpoint
    /// containment.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        FixedSegment2d capsuleAxis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetCapsuleDistanceInterval(
            this,
            capsuleAxis,
            radius,
            radiusExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered capsule.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetCapsuleIntersectionDistanceInterval(
            center,
            axisDirection,
            axisLength,
            radius,
            Fixed64.Zero,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered, radially expanded capsule and reports exact endpoint
    /// containment.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetCapsuleDistanceInterval(
            this,
            center,
            axisDirection,
            axisLength,
            radius,
            radiusExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Reconstructs the point at an authored physical distance along this
    /// segment using one exact chord interpolation per coordinate.
    /// </summary>
    /// <remarks>
    /// This method does not normalize <see cref="Delta"/>. It therefore retains
    /// representable chord components that would round away in a normalized
    /// direction. The final coordinates use round-half-to-even.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="totalDistance"/> is negative, or when
    /// <paramref name="distance"/> is outside [0, <paramref name="totalDistance"/>].
    /// </exception>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="totalDistance"/> is zero and this segment is
    /// not a point.
    /// </exception>
    public readonly Vector2d GetPointAtDistance(Fixed64 distance, Fixed64 totalDistance)
    {
        ValidateDistance(distance, totalDistance);
        if (distance == Fixed64.Zero)
            return Start;
        if (distance == totalDistance)
            return End;

        Signed192 distanceRaw = Signed192.Signed(distance.m_rawValue);
        Signed192 totalDistanceRaw = Signed192.Signed(totalDistance.m_rawValue);
        return new Vector2d(
            WideGeometry.InterpolateCoordinate(Start.X, End.X, distanceRaw, totalDistanceRaw),
            WideGeometry.InterpolateCoordinate(Start.Y, End.Y, distanceRaw, totalDistanceRaw));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly void ValidateDistance(Fixed64 distance, Fixed64 totalDistance)
    {
        ValidateTotalDistance(totalDistance);
        if (distance < Fixed64.Zero || distance > totalDistance)
            throw new ArgumentOutOfRangeException(nameof(distance));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly void ValidateTotalDistance(Fixed64 totalDistance)
    {
        if (totalDistance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(totalDistance));
        if (totalDistance == Fixed64.Zero && Start != End)
            throw new ArgumentException("Zero total distance requires a zero-length segment.", nameof(totalDistance));
    }
}
