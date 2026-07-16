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
    /// <remarks>
    /// This is ordinary component-wise <see cref="Fixed64"/> subtraction and
    /// therefore uses the public saturating vector-arithmetic contract. Use the
    /// segment query methods when endpoint differences may span the complete raw
    /// domain; their wider intermediate contract does not extend to this property.
    /// </remarks>
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
    /// Endpoint differences and projection products are evaluated across the
    /// complete raw domain before the parameter is clamped and rounded. A
    /// direction whose exact Q64.64 squared-length total is at most 2^31 raw
    /// units rounds to zero in Q32.32 and deterministically returns
    /// <see cref="Start"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ClosestPoint(Vector3d point) => Vector3d.ClosestPointOnLineSegment(point, Start, End);

    /// <summary>
    /// Computes the squared distance from the supplied point to this finite segment.
    /// </summary>
    /// <remarks>
    /// Component differences and their exact squared sum are evaluated across the
    /// complete raw domain before one final round-half-to-even conversion. Results
    /// outside the positive <see cref="Fixed64"/> range saturate to
    /// <see cref="Fixed64.MaxValue"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DistanceSquared(Vector3d point)
    {
        Vector3d closest = ClosestPoint(point);
        return Fixed64.RoundSquaredDistance(GetDifferenceDot(point, closest, point, closest));
    }

    /// <summary>
    /// Returns the closest finite points on this segment and another segment.
    /// </summary>
    /// <remarks>
    /// Endpoint differences, dot products, determinants, and parameter numerators
    /// are evaluated across the complete raw domain. A direction whose exact
    /// Q64.64 squared-length total is at most 2^31 raw units rounds to zero in
    /// Q32.32 and is treated as a point at its start. The established
    /// near-parallel policy compares the exact determinant magnitude with
    /// <see cref="Fixed64.Epsilon"/> before division. Parameters are rounded
    /// half-to-even and deterministically clamped to the closed interval [0, 1].
    /// When the mathematical zero-separation contact is an existing endpoint,
    /// both returned points preserve that endpoint bit-for-bit.
    /// </remarks>
    public readonly (Vector3d ThisPoint, Vector3d OtherPoint) GetClosestPoints(FixedSegment other)
    {
        Signed192 firstLengthSquared = GetDifferenceDot(End, Start, End, Start);
        Signed192 secondLengthSquared = GetDifferenceDot(
            other.End,
            other.Start,
            other.End,
            other.Start);

        if (WideGeometry.IsSquaredLengthDegenerate(firstLengthSquared))
        {
            if (WideGeometry.IsSquaredLengthDegenerate(secondLengthSquared))
                return (Start, other.Start);
            if (PointOnSegment(Start, other, secondLengthSquared))
                return (Start, Start);
            return (Start, other.ClosestPoint(Start));
        }

        if (WideGeometry.IsSquaredLengthDegenerate(secondLengthSquared))
        {
            if (PointOnSegment(other.Start, this, firstLengthSquared))
                return (other.Start, other.Start);
            return (Vector3d.ClosestPointOnLineSegment(other.Start, Start, End), other.Start);
        }

        Signed192 directionsDot = GetDifferenceDot(End, Start, other.End, other.Start);
        Signed192 firstDirectionDotDifference = GetDifferenceDot(End, Start, Start, other.Start);
        Signed192 secondDirectionDotDifference = GetDifferenceDot(
            other.End,
            other.Start,
            Start,
            other.Start);
        Signed320 determinant = WideArithmetic.MultiplySubtract(
            firstLengthSquared,
            secondLengthSquared,
            directionsDot,
            directionsDot);

        (Fixed64 firstParameter, Fixed64 secondParameter, byte endpointCandidates) = SolveClosestParameters(
            firstLengthSquared,
            directionsDot,
            secondLengthSquared,
            firstDirectionDotDifference,
            secondDirectionDotDifference,
            determinant);

        if ((endpointCandidates & 1) != 0 && PointOnSegment(Start, other, secondLengthSquared))
            return (Start, Start);
        if ((endpointCandidates & 2) != 0 && PointOnSegment(End, other, secondLengthSquared))
            return (End, End);
        if ((endpointCandidates & 4) != 0 && PointOnSegment(other.Start, this, firstLengthSquared))
            return (other.Start, other.Start);
        if ((endpointCandidates & 8) != 0 && PointOnSegment(other.End, this, firstLengthSquared))
            return (other.End, other.End);

        return (
            Interpolate(this, firstParameter),
            Interpolate(other, secondParameter));
    }

    private static (Fixed64 First, Fixed64 Second, byte EndpointCandidates) SolveClosestParameters(
        Signed192 firstLengthSquared,
        Signed192 directionsDot,
        Signed192 secondLengthSquared,
        Signed192 firstDirectionDotDifference,
        Signed192 secondDirectionDotDifference,
        Signed320 determinant)
    {
        Fixed64 firstParameter;
        bool firstParameterIsClamped;
        Signed320 firstNumerator = default;
        Signed320 secondNumerator = default;
        Signed320 secondDenominator = default;
        Signed192 narrowSecondNumerator = default;
        Signed192 narrowSecondDenominator = default;
        bool secondRatioIsWide;
        byte endpointCandidates = 0;
        bool isNearParallel = WideGeometry.IsSegmentDeterminantNearParallel(determinant);

        if (isNearParallel)
        {
            firstParameter = Fixed64.Zero;
            firstParameterIsClamped = true;
            endpointCandidates = 1;
            bool useDirectionsDot = directionsDot.Sign > 0
                && WideArithmetic.CompareMagnitude(directionsDot, secondLengthSquared) > 0;
            narrowSecondNumerator = useDirectionsDot
                ? firstDirectionDotDifference
                : secondDirectionDotDifference;
            narrowSecondDenominator = useDirectionsDot ? directionsDot : secondLengthSquared;
            secondRatioIsWide = false;
        }
        else
        {
            firstNumerator = WideArithmetic.MultiplySubtract(
                directionsDot,
                secondDirectionDotDifference,
                secondLengthSquared,
                firstDirectionDotDifference);
            int firstNumeratorComparison = firstNumerator.Sign < 0
                ? -1
                : WideArithmetic.CompareMagnitude(firstNumerator, determinant);

            if (firstNumerator.Sign < 0)
            {
                firstParameter = Fixed64.Zero;
                firstParameterIsClamped = true;
                endpointCandidates = 0;
                narrowSecondNumerator = secondDirectionDotDifference;
                narrowSecondDenominator = secondLengthSquared;
                secondRatioIsWide = false;
            }
            else if (firstNumeratorComparison > 0)
            {
                firstParameter = Fixed64.One;
                firstParameterIsClamped = true;
                endpointCandidates = 0;
                narrowSecondNumerator = WideArithmetic.AddSigned192(
                    secondDirectionDotDifference,
                    directionsDot);
                narrowSecondDenominator = secondLengthSquared;
                secondRatioIsWide = false;
            }
            else
            {
                firstParameter = default;
                firstParameterIsClamped = false;
                secondNumerator = WideArithmetic.MultiplySubtract(
                    firstLengthSquared,
                    secondDirectionDotDifference,
                    directionsDot,
                    firstDirectionDotDifference);
                secondDenominator = determinant;
                secondRatioIsWide = true;

                int secondNumeratorComparison = secondNumerator.Sign < 0
                    ? -1
                    : WideArithmetic.CompareMagnitude(secondNumerator, determinant);
                if (secondNumerator.Sign >= 0 && secondNumeratorComparison <= 0)
                {
                    if (firstNumerator.IsZero)
                        endpointCandidates |= 1;
                    else if (firstNumeratorComparison == 0)
                        endpointCandidates |= 2;
                    if (secondNumerator.IsZero)
                        endpointCandidates |= 4;
                    else if (secondNumeratorComparison == 0)
                        endpointCandidates |= 8;
                }
            }
        }

        int secondNumeratorSign = secondRatioIsWide
            ? secondNumerator.Sign
            : narrowSecondNumerator.Sign;
        if (secondNumeratorSign < 0)
            return (
                ClampParameter(
                    WideArithmetic.SubtractSigned192(default, firstDirectionDotDifference),
                    firstLengthSquared),
                Fixed64.Zero,
                isNearParallel ? (byte)4 : (byte)0);

        int secondRatioComparison = secondRatioIsWide
            ? WideArithmetic.CompareMagnitude(secondNumerator, secondDenominator)
            : WideArithmetic.CompareMagnitude(narrowSecondNumerator, narrowSecondDenominator);
        if (secondRatioComparison > 0)
            return (
                ClampParameter(
                    WideArithmetic.SubtractSigned192(directionsDot, firstDirectionDotDifference),
                    firstLengthSquared),
                Fixed64.One,
                isNearParallel ? (byte)8 : (byte)0);

        Fixed64 secondParameter;
        if (secondRatioIsWide)
        {
            _ = Fixed64.TryGetUnitIntervalRatio(
                secondNumerator,
                secondDenominator,
                out secondParameter);
        }
        else
        {
            _ = Fixed64.TryGetUnitIntervalRatio(
                narrowSecondNumerator,
                narrowSecondDenominator,
                out secondParameter);
        }
        if (!firstParameterIsClamped)
            _ = Fixed64.TryGetUnitIntervalRatio(firstNumerator, determinant, out firstParameter);

        return (firstParameter, secondParameter, endpointCandidates);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ClampParameter(Signed192 numerator, Signed192 denominator)
    {
        if (numerator.Sign <= 0)
            return Fixed64.Zero;
        if (WideArithmetic.CompareMagnitude(numerator, denominator) >= 0)
            return Fixed64.One;
        _ = Fixed64.TryGetUnitIntervalRatio(numerator, denominator, out Fixed64 parameter);
        return parameter;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 GetDifferenceDot(
        Vector3d leftEnd,
        Vector3d leftStart,
        Vector3d rightEnd,
        Vector3d rightStart)
    {
        return WideGeometry.GetDifferenceDotProduct3D(
            leftEnd.X, leftStart.X, leftEnd.Y, leftStart.Y, leftEnd.Z, leftStart.Z,
            rightEnd.X, rightStart.X, rightEnd.Y, rightStart.Y, rightEnd.Z, rightStart.Z);
    }

    private static bool PointOnSegment(
        Vector3d point,
        FixedSegment segment,
        Signed192 lengthSquared)
    {
        long pointX = point.X.m_rawValue;
        long pointY = point.Y.m_rawValue;
        long pointZ = point.Z.m_rawValue;
        if (pointX < Math.Min(segment.Start.X.m_rawValue, segment.End.X.m_rawValue)
            || pointX > Math.Max(segment.Start.X.m_rawValue, segment.End.X.m_rawValue)
            || pointY < Math.Min(segment.Start.Y.m_rawValue, segment.End.Y.m_rawValue)
            || pointY > Math.Max(segment.Start.Y.m_rawValue, segment.End.Y.m_rawValue)
            || pointZ < Math.Min(segment.Start.Z.m_rawValue, segment.End.Z.m_rawValue)
            || pointZ > Math.Max(segment.Start.Z.m_rawValue, segment.End.Z.m_rawValue))
        {
            return false;
        }

        Signed192 projection = GetDifferenceDot(
            point,
            segment.Start,
            segment.End,
            segment.Start);
        Signed192 pointDistanceSquared = GetDifferenceDot(
            point,
            segment.Start,
            point,
            segment.Start);
        return WideArithmetic.MultiplySubtract(
            lengthSquared,
            pointDistanceSquared,
            projection,
            projection).IsZero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d Interpolate(FixedSegment segment, Fixed64 parameter)
    {
        return new Vector3d(
            FixedMath.Lerp(segment.Start.X, segment.End.X, parameter),
            FixedMath.Lerp(segment.Start.Y, segment.End.Y, parameter),
            FixedMath.Lerp(segment.Start.Z, segment.End.Z, parameter));
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
