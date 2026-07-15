//=======================================================================
// FixedSegment.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp.Bounds;

/// <summary>
/// Represents a finite line segment in three-dimensional fixed-point space.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct FixedSegment : IEquatable<FixedSegment>
{
    #region Fields

    /// <summary>
    /// The start point of the segment.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Vector3d Start;

    /// <summary>
    /// The end point of the segment.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Vector3d End;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new segment from start and end points.
    /// </summary>
    [JsonConstructor]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedSegment(Vector3d start, Vector3d end)
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
    public Vector3d Delta
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
    /// The normalized axis-aligned box that contains this segment.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public FixedBoundBox Bounds
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => FixedBoundBox.FromMinMax(Start, End);
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
    public Vector3d ClosestPoint(Vector3d point) => Vector3d.ClosestPointOnLineSegment(point, Start, End);

    /// <summary>
    /// Computes the squared distance from the supplied point to this finite segment.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DistanceSquared(Vector3d point)
    {
        return Vector3d.DistanceSquared(point, ClosestPoint(point));
    }

    /// <summary>
    /// Returns the closest finite points on this segment and another segment.
    /// </summary>
    /// <remarks>
    /// A segment whose Q32.32 squared direction rounds to zero is treated as a
    /// point at its start. Non-degenerate inputs preserve the established finite
    /// segment parameter and clamping policy.
    /// </remarks>
    public readonly (Vector3d ThisPoint, Vector3d OtherPoint) GetClosestPoints(FixedSegment other)
    {
        Vector3d firstDirection = End - Start;
        Vector3d secondDirection = other.End - other.Start;
        Fixed64 firstLengthSquared = Vector3d.Dot(firstDirection, firstDirection);
        Fixed64 secondLengthSquared = Vector3d.Dot(secondDirection, secondDirection);

        if (firstLengthSquared == Fixed64.Zero)
        {
            return secondLengthSquared == Fixed64.Zero
                ? (Start, other.Start)
                : (Start, other.ClosestPoint(Start));
        }

        if (secondLengthSquared == Fixed64.Zero)
            return (Vector3d.ClosestPointOnLineSegment(other.Start, Start, End), other.Start);

        Vector3d startDifference = Start - other.Start;
        Fixed64 directionsDot = Vector3d.Dot(firstDirection, secondDirection);
        Fixed64 firstDirectionDotDifference = Vector3d.Dot(firstDirection, startDifference);
        Fixed64 secondDirectionDotDifference = Vector3d.Dot(secondDirection, startDifference);
        Fixed64 determinant = (firstLengthSquared * secondLengthSquared) - (directionsDot * directionsDot);

        (Fixed64 firstParameter, Fixed64 secondParameter) = SolveClosestParameters(
            firstLengthSquared,
            directionsDot,
            secondLengthSquared,
            firstDirectionDotDifference,
            secondDirectionDotDifference,
            determinant);

        if (firstParameter < Fixed64.Zero)
        {
            firstParameter = Fixed64.Zero;
            secondParameter = ClampParameter(secondDirectionDotDifference, secondLengthSquared);
        }
        else if (firstParameter > Fixed64.One)
        {
            firstParameter = Fixed64.One;
            secondParameter = ClampParameter(secondDirectionDotDifference + directionsDot, secondLengthSquared);
        }

        if (secondParameter < Fixed64.Zero)
        {
            secondParameter = Fixed64.Zero;
            firstParameter = ClampParameter(-firstDirectionDotDifference, firstLengthSquared);
        }
        else if (secondParameter > Fixed64.One)
        {
            secondParameter = Fixed64.One;
            firstParameter = ClampParameter(-firstDirectionDotDifference + directionsDot, firstLengthSquared);
        }

        return (
            Start + (firstParameter * firstDirection),
            other.Start + (secondParameter * secondDirection));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (Fixed64 First, Fixed64 Second) SolveClosestParameters(
        Fixed64 firstLengthSquared,
        Fixed64 directionsDot,
        Fixed64 secondLengthSquared,
        Fixed64 firstDirectionDotDifference,
        Fixed64 secondDirectionDotDifference,
        Fixed64 determinant)
    {
        if (determinant.Abs() < Fixed64.Epsilon)
        {
            Fixed64 secondParameter = directionsDot > secondLengthSquared
                ? firstDirectionDotDifference / directionsDot
                : secondDirectionDotDifference / secondLengthSquared;
            return (Fixed64.Zero, secondParameter);
        }

        return (
            ((directionsDot * secondDirectionDotDifference)
                - (secondLengthSquared * firstDirectionDotDifference)) / determinant,
            ((firstLengthSquared * secondDirectionDotDifference)
                - (directionsDot * firstDirectionDotDifference)) / determinant);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ClampParameter(Fixed64 numerator, Fixed64 denominator)
    {
        if (numerator < Fixed64.Zero)
            return Fixed64.Zero;
        if (numerator > denominator)
            return Fixed64.One;
        return numerator / denominator;
    }

    #endregion

    #region Deconstruction

    /// <summary>
    /// Deconstructs the segment into start and end points.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Vector3d start, out Vector3d end)
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
    public static bool operator ==(FixedSegment left, FixedSegment right) => left.Equals(right);

    /// <summary>
    /// Determines whether two segments have different ordered endpoints.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedSegment left, FixedSegment right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedSegment other)
    {
        return Start == other.Start && End == other.End;
    }

    /// <inheritdoc />
    public override bool Equals(object? obj)
    {
        return obj is FixedSegment other && Equals(other);
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
}
