using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a range of values with fixed precision.
    /// </summary>
    [Serializable]
    public struct FixedRange : IEquatable<FixedRange>
    {
        /// <summary>
        /// The smallest possible range.
        /// </summary>
        private static FixedRange _minRange = new FixedRange(FixedMath.MinValue, FixedMath.MinValue);
        public static FixedRange MinRange => _minRange;

        /// <summary>
        /// The largest possible range.
        /// </summary>
        private static FixedRange _maxRange = new FixedRange(FixedMath.MaxValue, FixedMath.MaxValue);
        public static FixedRange MaxRange => _maxRange;

        /// <summary>
        /// Gets the minimum value of the range.
        /// </summary>
        public Fixed64 Min { get; private set; }

        /// <summary>
        /// Gets the maximum value of the range.
        /// </summary>
        public Fixed64 Max { get; private set; }

        /// <summary>
        /// The length of the range, computed as Max - Min.
        /// </summary>
        public Fixed64 Length => Max - Min;

        /// <summary>
        /// The midpoint of the range.
        /// </summary>
        public Fixed64 MidPoint => (Min + Max) * FixedMath.Half;

        /// <summary>
        /// Initializes a new instance of the FixedRange structure with the specified minimum and maximum values.
        /// </summary>
        /// <param name="min">The minimum value of the range.</param>
        /// <param name="max">The maximum value of the range.</param>
        /// <param name="enforceOrder">If true, ensures that Min is less than or equal to Max.</param>

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
        /// Checks whether two FixedRange instances intersect.
        /// </summary>
        /// <param name="f1">The first range to compare.</param>
        /// <param name="f2">The second range to compare.</param>
        /// <returns>True if the ranges intersect; otherwise, false.</returns>
        public static bool Intersects(FixedRange f1, FixedRange f2)
        {
            FixedRange firstRange = f1.Min < f2.Min ? f1 : f2;
            FixedRange secondRange = firstRange.GetHashCode() == f1.GetHashCode() ? f2 : f1;
            return firstRange.Max >= secondRange.Min;
        }

        /// <summary>
        /// Checks whether the specified value is within the range.
        /// </summary>
        /// <param name="x">The value to check.</param>
        /// <returns>True if the value is within the range; otherwise, false.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool InRange(Fixed64 x)
        {
            return x >= Min && x < Max;
        }

        /// <summary>
        /// Checks whether the specified integer is within the range.
        /// </summary>
        /// <param name="x">The integer value to check.</param>
        /// <returns>True if the value is within the range; otherwise, false.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool InRange(int x)
        {
            return x >= (int)Min && x < (int)Max;
        }

        /// <summary>
        /// Checks whether the specified floating-point number is within the range.
        /// </summary>
        /// <param name="x">The floating-point value to check.</param>
        /// <returns>True if the value is within the range; otherwise, false.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool InRange(float x)
        {
            return (Fixed64)x >= Min && (Fixed64)x < Max;
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
        /// Checks whether this range overlaps with the specified range.
        /// </summary>
        /// <param name="other">The range to compare.</param>
        /// <returns>True if the ranges overlap; otherwise, false.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Overlaps(FixedRange other)
        {
            return Min <= other.Max && Max >= other.Min;
        }

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
                if (range1.Max < range2.Min) sign = -FixedMath.One;
                else sign = FixedMath.One;
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

            return overlap > FixedMath.Zero ? overlap : FixedMath.Zero;
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

        #region Convert

        /// <summary>
        /// Returns a string that represents the FixedRange instance, formatted as "Min - Max".
        /// </summary>
        public override string ToString()
        {
            return $"{Min.ToFormattedDouble()} - {Max.ToFormattedDouble()}";
        }

        #endregion

        #region Operators

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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
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