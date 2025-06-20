using MessagePack;
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a range of values with fixed precision.
    /// </summary>
    [Serializable]
    [MessagePackObject]
    public struct FixedRange : IEquatable<FixedRange>
    {
        #region Constants

        /// <summary>
        /// The smallest possible range.
        /// </summary>
        public static readonly FixedRange MinRange = new FixedRange(Fixed64.MIN_VALUE, Fixed64.MIN_VALUE);

        /// <summary>
        /// The largest possible range.
        /// </summary>
        public static readonly FixedRange MaxRange = new FixedRange(Fixed64.MAX_VALUE, Fixed64.MAX_VALUE);

        #endregion

        #region Fields

        /// <summary>
        /// Gets the minimum value of the range.
        /// </summary>
        [Key(0)]
        public Fixed64 Min;

        /// <summary>
        /// Gets the maximum value of the range.
        /// </summary>
        [Key(1)]
        public Fixed64 Max;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the FixedRange structure with the specified minimum and maximum values.
        /// </summary>
        /// <param name="min">The minimum value of the range.</param>
        /// <param name="max">The maximum value of the range.</param>
        /// <param name="enforceOrder">If true, ensures that Min is less than or equal to Max.</param>

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        #region Properties and Methods (Instance)

        /// <summary>
        /// The length of the range, computed as Max - Min.
        /// </summary>
        [IgnoreMember]
        public Fixed64 Length
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Max - Min;
        }

        /// <summary>
        /// The midpoint of the range.
        /// </summary>
        [IgnoreMember]
        public Fixed64 MidPoint
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (Min + Max) * Fixed64.Half;
        }

        /// <summary>
        /// Sets the minimum and maximum values for the range.
        /// </summary>
        /// <param name="min">The new minimum value.</param>
        /// <param name="max">The new maximum value.</param>
        public void SetMinMax(Fixed64 min, Fixed64 max)
        {
            Min = min;
            Max = max;
        }

        /// <summary>
        /// Adds a value to both the minimum and maximum of the range.
        /// </summary>
        /// <param name="val">The value to add.</param>
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

        /// <inheritdoc cref="InRange(Fixed64, bool)" />
        public bool InRange(double x, bool includeMax = false)
        {
            long xL = (long)Math.Round((double)x * FixedMath.ONE_L);
            return includeMax ? xL >= Min.m_rawValue && xL <= Max.m_rawValue : xL >= Min.m_rawValue && xL < Max.m_rawValue;
        }

        /// <summary>
        /// Checks whether this range overlaps with the specified range, ensuring no adjacent edges are considered overlaps.
        /// </summary>
        /// <param name="other">The range to compare.</param>
        /// <returns>True if the ranges overlap; otherwise, false.</returns>
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
        /// <param name="range1">The first range.</param>
        /// <param name="range2">The second range.</param>
        /// <param name="sign">The direction between ranges (-1 or 1).</param>
        /// <returns>True if the ranges don't overlap, false if they do.</returns>
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
        /// <param name="rangeA">The first range.</param>
        /// <param name="rangeB">The second range.</param>
        /// <returns>The depth of the overlap between the ranges.</returns>
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
        /// <param name="origin">The origin vector.</param>
        /// <param name="range1">The first range.</param>
        /// <param name="range2">The second range.</param>
        /// <param name="limit">The overlap limit to check.</param>
        /// <param name="sign">The direction sign to consider.</param>
        /// <param name="output">The overlap vector and depth, if any.</param>
        /// <returns>True if overlap occurs and is below the limit, otherwise false.</returns>
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedRange operator +(FixedRange left, FixedRange right)
        {
            return new FixedRange(left.Min + right.Min, left.Max + right.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedRange operator -(FixedRange left, FixedRange right)
        {
            return new FixedRange(left.Min - right.Min, left.Max - right.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(FixedRange left, FixedRange right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(FixedRange left, FixedRange right)
        {
            return !left.Equals(right);
        }

        #endregion

        #region Conversion

        /// <summary>
        /// Returns a string that represents the FixedRange instance, formatted as "Min - Max".
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return $"{Min.ToFormattedDouble()} - {Max.ToFormattedDouble()}";
        }

        #endregion

        #region Equality and HashCode Overrides

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object? obj)
        {
            return obj is FixedRange other && Equals(other);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(FixedRange other)
        {
            return other.Min == Min && other.Max == Max;
        }

        /// <summary>
        /// Computes the hash code for the FixedRange instance.
        /// </summary>
        /// <returns>The hash code of the range.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return Min.GetHashCode() ^ Max.GetHashCode();
        }

        #endregion
    }
}