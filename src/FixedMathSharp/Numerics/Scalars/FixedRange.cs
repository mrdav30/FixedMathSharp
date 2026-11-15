//=======================================================================
// FixedRange.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp;

/// <summary>
/// Represents a range of values with fixed precision.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct FixedRange : IEquatable<FixedRange>, IFormattable
#if NET10_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Static Readonly Fields

    /// <summary>
    /// The degenerate range at <see cref="Fixed64.MinValue"/>.
    /// </summary>
    public static readonly FixedRange MinRange = new(Fixed64.MinValue, Fixed64.MinValue);

    /// <summary>
    /// The degenerate range at <see cref="Fixed64.MaxValue"/>.
    /// </summary>
    public static readonly FixedRange MaxRange = new(Fixed64.MaxValue, Fixed64.MaxValue);

    #endregion

    #region Fields

    /// <summary>
    /// Gets the minimum value of the range.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Fixed64 Min;

    /// <summary>
    /// Gets the maximum value of the range.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Fixed64 Max;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new instance of the FixedRange structure with the specified minimum and maximum values.
    /// </summary>
    /// <param name="min">The proposed lower endpoint.</param>
    /// <param name="max">The proposed upper endpoint.</param>
    /// <param name="enforceOrder">When true, the endpoints are sorted. Otherwise, they are stored in the supplied order.</param>
    public FixedRange(Fixed64 min, Fixed64 max, bool enforceOrder = true)
    {
        if (enforceOrder)
        {
            Min = min < max ? min : max;
            Max = min < max ? max : min;
        }
        else
        {
            Min = min;
            Max = max;
        }
    }

    #endregion

    #region Properties

    /// <summary>
    /// The exact signed length of the range, computed as <see cref="Max"/> minus
    /// <see cref="Min"/>.
    /// </summary>
    /// <exception cref="OverflowException">
    /// The signed endpoint difference is outside the representable scalar domain.
    /// </exception>
    /// <value>The signed end-to-start difference. An inverted range has a negative length.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Length
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            if (!Fixed64.TrySubtract(Max, Min, out Fixed64 length))
                throw new OverflowException("The range length is outside the representable Fixed64 domain.");

            return length;
        }
    }

    /// <summary>
    /// The nearest-even Q32.32 midpoint of the range.
    /// </summary>
    /// <value>The representable value closest to the arithmetic mean of the endpoints, with ties to even.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 MidPoint
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => FixedMath.Midpoint(Min, Max);
    }

    #endregion

    #region Methods (Instance)

    /// <summary>
    /// Sets the minimum and maximum values for the range.
    /// </summary>
    /// <param name="min">The value assigned to <see cref="Min"/>.</param>
    /// <param name="max">The value assigned to <see cref="Max"/>.</param>
    /// <remarks>The endpoints are assigned in the supplied order.</remarks>
    public void SetMinMax(Fixed64 min, Fixed64 max)
    {
        Min = min;
        Max = max;
    }

    /// <summary>
    /// Adds a value to both the minimum and maximum of the range.
    /// </summary>
    /// <param name="val">The offset in the same units as the range endpoints.</param>
    /// <remarks>The two updated endpoints saturate independently.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddInPlace(Fixed64 val)
    {
        Min += val;
        Max += val;
    }

    /// <summary>
    /// Determines whether the specified value is within the range, with an option to include or exclude the upper bound.
    /// </summary>
    /// <param name="x">The value to check.</param>
    /// <param name="includeMax">If true, the upper bound (Max) is included in the range check; otherwise, the upper bound is exclusive. Default is false (exclusive).</param>
    /// <returns>True if the value is within the range; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool InRange(Fixed64 x, bool includeMax = false)
    {
        return includeMax ? x >= Min && x <= Max : x >= Min && x < Max;
    }

    /// <summary>
    /// Checks whether this range overlaps with the specified range, ensuring no adjacent edges are considered overlaps.
    /// </summary>
    /// <param name="other">The range to compare.</param>
    /// <returns><see langword="true"/> if this minimum is below the other maximum and this maximum is above the other minimum, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Overlaps(FixedRange other)
    {
        return Min < other.Max && Max > other.Min;
    }

    #endregion

    #region Range Operations

    /// <summary>
    /// Determines the direction from one range to another.
    /// If they don't overlap, returns -1 or 1 depending on the relative position.
    /// </summary>
    /// <param name="range1">The first range, with ordered endpoints.</param>
    /// <param name="range2">The second range, with ordered endpoints.</param>
    /// <param name="sign">Null when the ranges overlap. Otherwise, −1 when the first maximum is strictly below the second minimum and +1 in all other cases.</param>
    /// <returns><see langword="true"/> when <see cref="Overlaps(FixedRange)"/> returns false for the pair, otherwise <see langword="false"/>.</returns>
    public static bool GetDirection(FixedRange range1, FixedRange range2, out Fixed64? sign)
    {
        sign = null;
        if (!range1.Overlaps(range2))
        {
            if (range1.Max < range2.Min) sign = -Fixed64.One;
            else sign = Fixed64.One;
            return true;
        }
        return false;
    }

    /// <summary>
    /// Calculates the overlap depth between two ranges.
    /// Assumes the ranges are sorted (min and max are correctly assigned).
    /// </summary>
    /// <param name="rangeA">The first range, with ordered endpoints.</param>
    /// <param name="rangeB">The second range, with ordered endpoints.</param>
    /// <returns>The overlap length for partial overlap, zero for disjoint ranges, or the distance from the containing range’s minimum to the contained range’s maximum.</returns>
    /// <remarks>Endpoint differences saturate at the representable endpoints.</remarks>
    public static Fixed64 ComputeOverlapDepth(FixedRange rangeA, FixedRange rangeB)
    {
        // Check if one range is completely within the other
        bool isRangeAInsideB = rangeA.Min >= rangeB.Min && rangeA.Max <= rangeB.Max;
        bool isRangeBInsideA = rangeB.Min >= rangeA.Min && rangeB.Max <= rangeA.Max;
        if (isRangeAInsideB)
            return rangeA.Max - rangeB.Min; // The size of rangeA
        else if (isRangeBInsideA)
            return rangeB.Max - rangeA.Min; // The size of rangeB

        // Calculate overlap between the two ranges
        Fixed64 overlapEnd = FixedMath.Min(rangeA.Max, rangeB.Max);
        Fixed64 overlapStart = FixedMath.Max(rangeA.Min, rangeB.Min);
        Fixed64 overlap = overlapEnd - overlapStart;

        return overlap > Fixed64.Zero ? overlap : Fixed64.Zero;
    }

    /// <summary>
    /// Checks for overlap between two ranges and calculates the vector of overlap depth.
    /// </summary>
    /// <param name="origin">The vector to scale by the depth and sign.</param>
    /// <param name="range1">The first range, with ordered endpoints.</param>
    /// <param name="range2">The second range, with ordered endpoints.</param>
    /// <param name="limit">The exclusive upper limit on the depth.</param>
    /// <param name="sign">The signed scale applied to the depth vector, normally −1 or +1.</param>
    /// <param name="output">The scaled vector and depth on success, or null on failure.</param>
    /// <returns><see langword="true"/> if the computed depth is strictly below the limit, even if the depth is zero. Otherwise, <see langword="false"/>.</returns>
    public static bool CheckOverlap(Vector3d origin, FixedRange range1, FixedRange range2, Fixed64 limit, Fixed64 sign, out (Vector3d Vector, Fixed64 Depth)? output)
    {
        output = null;
        Fixed64 overlap = ComputeOverlapDepth(range1, range2);

        // If the overlap is smaller than the current minimum, update the minimum
        if (overlap < limit)
        {
            output = (origin * overlap * sign, overlap);
            return true;
        }
        return false;
    }

    #endregion

    #region Operators

    /// <summary>
    /// Adds two FixedRange instances by summing their minimum and maximum values.
    /// </summary>
    /// <param name="left">The first FixedRange to add.</param>
    /// <param name="right">The second FixedRange to add.</param>
    /// <returns>A new FixedRange whose Min is the sum of the Min values and whose Max is the sum of the Max values of the
    /// specified ranges.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedRange operator +(FixedRange left, FixedRange right)
    {
        return new FixedRange(left.Min + right.Min, left.Max + right.Max);
    }

    /// <summary>
    /// Subtracts the minimum and maximum values of one FixedRange from another and returns the resulting FixedRange.
    /// </summary>
    /// <param name="left">The FixedRange instance to subtract from.</param>
    /// <param name="right">The FixedRange instance whose values are subtracted.</param>
    /// <returns>A FixedRange whose Min and Max values are the result of subtracting the corresponding values of right from left.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedRange operator -(FixedRange left, FixedRange right)
    {
        return new FixedRange(left.Min - right.Min, left.Max - right.Max);
    }

    /// <summary>Tests two values for equality.</summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if the corresponding stored endpoints are equal, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedRange left, FixedRange right) => left.Equals(right);

    /// <summary>Tests two values for inequality.</summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if a corresponding endpoint differs, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedRange left, FixedRange right) => !left.Equals(right);

    #endregion

    #region Conversion

    /// <summary>
    /// Returns a string that represents the FixedRange instance, formatted as "Min - Max".
    /// </summary>
    /// <returns>Both endpoints separated by <c> - </c>, using the default invariant numeric format.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString() => ToString(null, CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns a string that represents the FixedRange instance, formatted as "Min - Max".
    /// </summary>
    /// <param name="format">The numeric format for both endpoints. Null or empty selects the default format.</param>
    /// <param name="formatProvider">The provider used for both endpoints. Null selects the invariant culture.</param>
    /// <returns>The formatted endpoints separated by <c> - </c>.</returns>
    /// <exception cref="FormatException">The numeric format is invalid.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToString(string? format, IFormatProvider? formatProvider)
    {
        FixedRange value = this;
        return FixedDiagnosticsFormatter.ToString((Span<char> destination, out int charsWritten) =>
            value.TryFormat(destination, out charsWritten, format.AsSpan(), formatProvider));
    }

    /// <summary>
    /// Formats this range into the provided destination buffer.
    /// </summary>
    /// <param name="destination">Caller-owned output span. The method writes from its start and may leave a partial prefix on failure.</param>
    /// <param name="charsWritten">The number of characters written on success, or zero when the destination is too small.</param>
    /// <param name="format">The numeric format used for both endpoints. An empty span selects the default format.</param>
    /// <param name="provider">The provider used for both endpoints. Null selects the invariant culture.</param>
    /// <returns><see langword="true"/> if both endpoints and the separator fit, otherwise <see langword="false"/>.</returns>
    /// <exception cref="FormatException">The numeric format is invalid.</exception>
    public bool TryFormat(
        Span<char> destination,
        out int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        int written = 0;
        if (!FixedDiagnosticsFormatter.Append(Min, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(" - ", destination, ref written) ||
            !FixedDiagnosticsFormatter.Append(Max, destination, ref written, format, provider))
        {
            charsWritten = 0;
            return false;
        }

        charsWritten = written;
        return true;
    }

    #endregion

    #region Equality and HashCode Overrides

    /// <summary>Compares this value with an object.</summary>
    /// <param name="obj">The object to compare.</param>
    /// <returns><see langword="true"/> if <paramref name="obj"/> is a <see cref="FixedRange"/> and the corresponding stored endpoints are equal, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj)
    {
        return obj is FixedRange other && Equals(other);
    }

    /// <summary>Compares this value with another instance.</summary>
    /// <param name="other">The value to compare.</param>
    /// <returns><see langword="true"/> if the corresponding stored endpoints are equal, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedRange other)
    {
        return other.Min == Min && other.Max == Max;
    }

    /// <summary>Computes the hash code for this value.</summary>
    /// <returns>The hash codes of the two endpoints combined using exclusive OR.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode()
    {
        return Min.GetHashCode() ^ Max.GetHashCode();
    }

    #endregion
}
