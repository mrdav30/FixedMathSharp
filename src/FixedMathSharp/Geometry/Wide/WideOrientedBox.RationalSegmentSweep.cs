//=======================================================================
// WideOrientedBox.RationalSegmentSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Wide-oriented box rational segment sweep operations.
/// </content>
internal static partial class WideOrientedBox
{
    // A rigid triangle vertex uses fewer than 192 numerator bits over a
    // 126-bit quaternion basis denominator. Finite-Y clipping raises a point
    // to fewer than 386/319 bits. Cross-multiplying two clipped endpoints
    // therefore needs at most 706 bits; a line violation needs 1094 bits and
    // its squared half-step comparison needs fewer than 2252 bits.
    private const int TriangleSweepMagnitudeWords = 36;

    private static bool TryGetRationalSegmentSweepDistance(
        SweepRationalPoint first,
        SweepRationalPoint second,
        Vector2d start,
        Vector2d direction,
        Fixed64 maximumDistance,
        Fixed64 radius,
        out Fixed64 distance,
        out Fixed64 segmentParameter)
    {
        Signed832 edgeX = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                second.X,
                first.Denominator),
            WideArithmetic.MultiplySigned576ToSigned832(
                first.X,
                second.Denominator));
        Signed832 edgeZ = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                second.Z,
                first.Denominator),
            WideArithmetic.MultiplySigned576ToSigned832(
                first.Z,
                second.Denominator));
        if (edgeX.IsZero && edgeZ.IsZero)
        {
            distance = default;
            segmentParameter = default;
            return false;
        }

        Signed576 startX = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                first.Denominator,
                Signed192.Raw(start.X)),
            first.X);
        Signed576 startZ = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                first.Denominator,
                Signed192.Raw(start.Y)),
            first.Z);
        Span<ulong> startCross =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        GetCrossMagnitude(
            edgeX,
            edgeZ,
            startX,
            startZ,
            startCross,
            out int startCrossSign);

        Signed192 directionX = Signed192.Raw(direction.X);
        Signed192 directionZ = Signed192.Raw(direction.Y);
        Span<ulong> velocityCore =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        GetCrossMagnitude(
            edgeX,
            edgeZ,
            directionX,
            directionZ,
            velocityCore,
            out int velocitySign);
        Span<ulong> denominatorMagnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(
            first.Denominator,
            denominatorMagnitude);
        Span<ulong> velocity =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(
            velocityCore,
            denominatorMagnitude,
            velocity);

        Span<ulong> edgeSquared =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        GetEdgeSquaredMagnitude(edgeX, edgeZ, edgeSquared);
        if (CompareRationalSegmentLineAt(
                startCross,
                startCrossSign,
                velocity,
                velocitySign,
                edgeSquared,
                denominatorMagnitude,
                radius,
                default,
                SweepRawScale) <= 0
            && TryGetRationalSegmentParameter(
                first,
                second,
                edgeX,
                edgeZ,
                edgeSquared,
                start,
                direction,
                Fixed64.Zero,
                out segmentParameter))
        {
            distance = Fixed64.Zero;
            return true;
        }

        if (maximumDistance == Fixed64.Zero
            || startCrossSign == 0
            || velocitySign == 0
            || startCrossSign == velocitySign)
        {
            distance = default;
            segmentParameter = default;
            return false;
        }

        Span<ulong> startAtUnit =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> velocityAtMaximum =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> oneMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(SweepRawScale, out oneMagnitude[2], out oneMagnitude[1], out oneMagnitude[0]);
        Span<ulong> maximumMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(
            Signed192.Signed(maximumDistance.m_rawValue),
            out maximumMagnitude[2],
            out maximumMagnitude[1],
            out maximumMagnitude[0]);
        MultiplyMagnitudes(startCross, oneMagnitude, startAtUnit);
        MultiplyMagnitudes(velocity, maximumMagnitude, velocityAtMaximum);
        bool closestBeyondMaximum =
            CompareMagnitudes(startAtUnit, velocityAtMaximum) > 0;
        if (closestBeyondMaximum
            && CompareRationalSegmentLineAt(
                startCross,
                startCrossSign,
                velocity,
                velocitySign,
                edgeSquared,
                denominatorMagnitude,
                radius,
                Signed192.Signed(maximumDistance.m_rawValue),
                SweepRawScale) > 0)
        {
            distance = default;
            segmentParameter = default;
            return false;
        }

        long candidateRaw = maximumDistance.m_rawValue;
        if (!closestBeyondMaximum)
        {
            long crossingLow = 0L;
            long crossingHigh = candidateRaw;
            while (crossingLow < crossingHigh)
            {
                long middle =
                    crossingLow + ((crossingHigh - crossingLow) >> 1);
                if (CompareApproachingLineAt(
                        startCross,
                        velocity,
                        middle,
                        oneMagnitude) <= 0)
                {
                    crossingHigh = middle;
                }
                else
                {
                    crossingLow = middle + 1L;
                }
            }

            candidateRaw = crossingLow;
        }

        // A nonzero approaching line crosses no earlier than raw tick one.
        long high = candidateRaw - 1L;
        if (CompareRationalSegmentLineAtHalf(
                startCross,
                startCrossSign,
                velocity,
                velocitySign,
                edgeSquared,
                denominatorMagnitude,
                radius,
                high) > 0)
        {
            distance = Fixed64.FromRaw(candidateRaw);
        }
        else
        {
            long low = 0L;
            while (low < high)
            {
                long middle = low + ((high - low) >> 1);
                if (CompareRationalSegmentLineAtHalf(
                        startCross,
                        startCrossSign,
                        velocity,
                        velocitySign,
                        edgeSquared,
                        denominatorMagnitude,
                        radius,
                        middle) <= 0)
                {
                    high = middle;
                }
                else
                {
                    low = middle + 1L;
                }
            }

            int midpointComparison =
                CompareRationalSegmentLineAtHalf(
                    startCross,
                    startCrossSign,
                    velocity,
                    velocitySign,
                    edgeSquared,
                    denominatorMagnitude,
                    radius,
                    low);
            distance = Fixed64.FromRaw(
                midpointComparison == 0 && (low & 1L) != 0L
                    ? low + 1L
                    : low);
        }

        if (TryGetRationalSegmentParameter(
                first,
                second,
                edgeX,
                edgeZ,
                edgeSquared,
                start,
                direction,
                distance,
                out segmentParameter))
        {
            return true;
        }

        distance = default;
        segmentParameter = default;
        return false;
    }

    private static int CompareApproachingLineAt(
        ReadOnlySpan<ulong> startCross,
        ReadOnlySpan<ulong> velocity,
        long distanceRaw,
        ReadOnlySpan<ulong> oneMagnitude)
    {
        Span<ulong> distanceMagnitude = stackalloc ulong[1]
        {
            unchecked((ulong)distanceRaw),
        };
        Span<ulong> startTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> velocityTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(startCross, oneMagnitude, startTerm);
        MultiplyMagnitudes(
            velocity,
            distanceMagnitude,
            velocityTerm);
        return CompareMagnitudes(startTerm, velocityTerm);
    }

    private static int CompareRationalSegmentLineAtHalf(
        ReadOnlySpan<ulong> startCross,
        int startCrossSign,
        ReadOnlySpan<ulong> velocity,
        int velocitySign,
        ReadOnlySpan<ulong> edgeSquared,
        ReadOnlySpan<ulong> denominator,
        Fixed64 radius,
        long lowerRaw)
    {
        Signed192 lower = Signed192.Signed(lowerRaw);
        Signed192 midpoint = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(lower, lower),
            SweepOneCoefficient);
        return CompareRationalSegmentLineAt(
            startCross,
            startCrossSign,
            velocity,
            velocitySign,
            edgeSquared,
            denominator,
            radius,
            midpoint,
            SweepDoubleRawScale);
    }

    private static int CompareRationalSegmentLineAt(
        ReadOnlySpan<ulong> startCross,
        int startCrossSign,
        ReadOnlySpan<ulong> velocity,
        int velocitySign,
        ReadOnlySpan<ulong> edgeSquared,
        ReadOnlySpan<ulong> denominator,
        Fixed64 radius,
        Signed192 timeNumerator,
        Signed192 timeDenominator)
    {
        Span<ulong> timeNumeratorMagnitude = stackalloc ulong[3];
        Span<ulong> timeDenominatorMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(
            timeNumerator,
            out timeNumeratorMagnitude[2],
            out timeNumeratorMagnitude[1],
            out timeNumeratorMagnitude[0]);
        WideArithmetic.GetMagnitude(
            timeDenominator,
            out timeDenominatorMagnitude[2],
            out timeDenominatorMagnitude[1],
            out timeDenominatorMagnitude[0]);
        Span<ulong> firstTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> secondTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> line =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(
            startCross,
            timeDenominatorMagnitude,
            firstTerm);
        MultiplyMagnitudes(
            velocity,
            timeNumeratorMagnitude,
            secondTerm);
        CombineSignedMagnitudes(
            firstTerm,
            startCrossSign,
            secondTerm,
            velocitySign * timeNumerator.Sign,
            line,
            out _);
        Span<ulong> leftSquared =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(line, line, leftSquared);

        Span<ulong> radiusMagnitude = stackalloc ulong[1]
        {
            unchecked((ulong)radius.m_rawValue),
        };
        Span<ulong> right =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> scratch =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(edgeSquared, denominator, right);
        MultiplyMagnitudes(right, denominator, scratch);
        MultiplyMagnitudes(scratch, radiusMagnitude, right);
        MultiplyMagnitudes(right, radiusMagnitude, scratch);
        MultiplyMagnitudes(
            scratch,
            timeDenominatorMagnitude,
            right);
        MultiplyMagnitudes(
            right,
            timeDenominatorMagnitude,
            scratch);
        return CompareMagnitudes(leftSquared, scratch);
    }

    private static bool TryGetRationalSegmentParameter(
        SweepRationalPoint first,
        SweepRationalPoint second,
        Signed832 edgeX,
        Signed832 edgeZ,
        ReadOnlySpan<ulong> edgeSquared,
        Vector2d start,
        Vector2d direction,
        Fixed64 distance,
        out Fixed64 parameter)
    {
        Signed576 startX = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                first.Denominator,
                Signed192.Raw(start.X)),
            first.X);
        Signed576 startZ = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                first.Denominator,
                Signed192.Raw(start.Y)),
            first.Z);
        Signed576 directionX = WideArithmetic.MultiplySigned576(
            first.Denominator,
            Signed192.Raw(direction.X));
        Signed576 directionZ = WideArithmetic.MultiplySigned576(
            first.Denominator,
            Signed192.Raw(direction.Y));
        Signed576 scaledX = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(startX, SweepRawScale),
            WideArithmetic.MultiplySigned576(
                directionX,
                Signed192.Signed(distance.m_rawValue)));
        Signed576 scaledZ = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(startZ, SweepRawScale),
            WideArithmetic.MultiplySigned576(
                directionZ,
                Signed192.Signed(distance.m_rawValue)));

        Span<ulong> dot = stackalloc ulong[TriangleSweepMagnitudeWords];
        GetDotMagnitude(
            edgeX,
            edgeZ,
            scaledX,
            scaledZ,
            dot,
            out int dotSign);
        if (dotSign < 0)
        {
            parameter = default;
            return false;
        }

        Span<ulong> secondDenominator = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(
            second.Denominator,
            secondDenominator);
        Span<ulong> numerator =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(dot, secondDenominator, numerator);
        Span<ulong> oneMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(
            SweepRawScale,
            out oneMagnitude[2],
            out oneMagnitude[1],
            out oneMagnitude[0]);
        Span<ulong> denominator =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(edgeSquared, oneMagnitude, denominator);
        if (CompareMagnitudes(numerator, denominator) > 0)
        {
            parameter = default;
            return false;
        }

        parameter = Fixed64.GetUnitIntervalRatio(
            numerator,
            denominator);
        return true;
    }

    private static void GetEdgeSquaredMagnitude(
        Signed832 edgeX,
        Signed832 edgeZ,
        Span<ulong> result)
    {
        Span<ulong> x = stackalloc ulong[13];
        Span<ulong> z = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(edgeX, x);
        WideArithmetic.GetMagnitude(edgeZ, z);
        Span<ulong> xSquared =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> zSquared =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(x, x, xSquared);
        MultiplyMagnitudes(z, z, zSquared);
        AddMagnitudes(xSquared, zSquared, result);
    }

    private static void GetCrossMagnitude(
        Signed832 firstX,
        Signed832 firstZ,
        Signed576 secondX,
        Signed576 secondZ,
        Span<ulong> result,
        out int sign)
    {
        Span<ulong> firstXMag = stackalloc ulong[13];
        Span<ulong> firstZMag = stackalloc ulong[13];
        Span<ulong> secondXMag = stackalloc ulong[9];
        Span<ulong> secondZMag = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(firstX, firstXMag);
        WideArithmetic.GetMagnitude(firstZ, firstZMag);
        WideArithmetic.GetMagnitude(secondX, secondXMag);
        WideArithmetic.GetMagnitude(secondZ, secondZMag);
        GetCrossMagnitude(
            firstXMag,
            firstX.Sign,
            firstZMag,
            firstZ.Sign,
            secondXMag,
            secondX.Sign,
            secondZMag,
            secondZ.Sign,
            result,
            out sign);
    }

    private static void GetCrossMagnitude(
        Signed832 firstX,
        Signed832 firstZ,
        Signed192 secondX,
        Signed192 secondZ,
        Span<ulong> result,
        out int sign)
    {
        Span<ulong> firstXMag = stackalloc ulong[13];
        Span<ulong> firstZMag = stackalloc ulong[13];
        Span<ulong> secondXMag = stackalloc ulong[3];
        Span<ulong> secondZMag = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(firstX, firstXMag);
        WideArithmetic.GetMagnitude(firstZ, firstZMag);
        WideArithmetic.GetMagnitude(
            secondX,
            out secondXMag[2],
            out secondXMag[1],
            out secondXMag[0]);
        WideArithmetic.GetMagnitude(
            secondZ,
            out secondZMag[2],
            out secondZMag[1],
            out secondZMag[0]);
        GetCrossMagnitude(
            firstXMag,
            firstX.Sign,
            firstZMag,
            firstZ.Sign,
            secondXMag,
            secondX.Sign,
            secondZMag,
            secondZ.Sign,
            result,
            out sign);
    }

    private static void GetCrossMagnitude(
        ReadOnlySpan<ulong> firstX,
        int firstXSign,
        ReadOnlySpan<ulong> firstZ,
        int firstZSign,
        ReadOnlySpan<ulong> secondX,
        int secondXSign,
        ReadOnlySpan<ulong> secondZ,
        int secondZSign,
        Span<ulong> result,
        out int sign)
    {
        Span<ulong> firstTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> secondTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(firstX, secondZ, firstTerm);
        MultiplyMagnitudes(firstZ, secondX, secondTerm);
        CombineSignedMagnitudes(
            firstTerm,
            firstXSign * secondZSign,
            secondTerm,
            -(firstZSign * secondXSign),
            result,
            out sign);
    }

    private static void GetDotMagnitude(
        Signed832 firstX,
        Signed832 firstZ,
        Signed576 secondX,
        Signed576 secondZ,
        Span<ulong> result,
        out int sign)
    {
        Span<ulong> firstXMag = stackalloc ulong[13];
        Span<ulong> firstZMag = stackalloc ulong[13];
        Span<ulong> secondXMag = stackalloc ulong[9];
        Span<ulong> secondZMag = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(firstX, firstXMag);
        WideArithmetic.GetMagnitude(firstZ, firstZMag);
        WideArithmetic.GetMagnitude(secondX, secondXMag);
        WideArithmetic.GetMagnitude(secondZ, secondZMag);
        Span<ulong> firstTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> secondTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(firstXMag, secondXMag, firstTerm);
        MultiplyMagnitudes(firstZMag, secondZMag, secondTerm);
        CombineSignedMagnitudes(
            firstTerm,
            firstX.Sign * secondX.Sign,
            secondTerm,
            firstZ.Sign * secondZ.Sign,
            result,
            out sign);
    }

    private static void CombineSignedMagnitudes(
        ReadOnlySpan<ulong> first,
        int firstSign,
        ReadOnlySpan<ulong> second,
        int secondSign,
        Span<ulong> result,
        out int sign)
    {
        if (firstSign == 0)
        {
            CopyMagnitude(second, result);
            sign = secondSign;
            return;
        }
        if (secondSign == 0)
        {
            CopyMagnitude(first, result);
            sign = firstSign;
            return;
        }
        if (firstSign == secondSign)
        {
            AddMagnitudes(first, second, result);
            sign = firstSign;
            return;
        }

        int comparison = CompareMagnitudes(first, second);
        if (comparison == 0)
        {
            result.Clear();
            sign = 0;
            return;
        }
        if (comparison > 0)
        {
            SubtractMagnitudes(first, second, result);
            sign = firstSign;
            return;
        }

        SubtractMagnitudes(second, first, result);
        sign = secondSign;
    }

    private static void MultiplyMagnitudes(
        ReadOnlySpan<ulong> first,
        ReadOnlySpan<ulong> second,
        Span<ulong> result)
    {
        result.Clear();
        int firstLength = GetActiveMagnitudeLength(first);
        int secondLength = GetActiveMagnitudeLength(second);
        for (int firstIndex = 0; firstIndex < firstLength; firstIndex++)
        {
            for (int secondIndex = 0;
                 secondIndex < secondLength;
                 secondIndex++)
            {
                int resultIndex = firstIndex + secondIndex;
                Fixed64.Multiply64To128(
                    first[firstIndex],
                    second[secondIndex],
                    out ulong high,
                    out ulong low);
                AddMagnitudeWord(result, resultIndex, low);
                AddMagnitudeWord(result, resultIndex + 1, high);
            }
        }
    }

    private static void AddMagnitudeWord(
        Span<ulong> magnitude,
        int index,
        ulong value)
    {
        while (value != 0UL)
        {
            ulong previous = magnitude[index];
            magnitude[index] = unchecked(previous + value);
            value = magnitude[index] < previous ? 1UL : 0UL;
            index++;
        }
    }

    private static void AddMagnitudes(
        ReadOnlySpan<ulong> first,
        ReadOnlySpan<ulong> second,
        Span<ulong> result)
    {
        result.Clear();
        ulong carry = 0UL;
        for (int index = 0;
            index < TriangleSweepMagnitudeWords;
            index++)
        {
            ulong firstWord = first[index];
            ulong secondWord = second[index];
            ulong sum = unchecked(firstWord + secondWord);
            ulong nextCarry = sum < firstWord ? 1UL : 0UL;
            ulong withCarry = unchecked(sum + carry);
            nextCarry |= withCarry < sum ? 1UL : 0UL;
            result[index] = withCarry;
            carry = nextCarry;
        }
    }

    private static void SubtractMagnitudes(
        ReadOnlySpan<ulong> minuend,
        ReadOnlySpan<ulong> subtrahend,
        Span<ulong> result)
    {
        result.Clear();
        ulong borrow = 0UL;
        for (int index = 0;
            index < TriangleSweepMagnitudeWords;
            index++)
        {
            ulong left = minuend[index];
            ulong right = subtrahend[index];
            ulong difference = unchecked(left - right);
            ulong nextBorrow = left < right ? 1UL : 0UL;
            ulong withBorrow = unchecked(difference - borrow);
            nextBorrow |= difference < borrow ? 1UL : 0UL;
            result[index] = withBorrow;
            borrow = nextBorrow;
        }
    }

    private static int CompareMagnitudes(
        ReadOnlySpan<ulong> first,
        ReadOnlySpan<ulong> second)
    {
        for (int index = TriangleSweepMagnitudeWords - 1;
            index >= 0;
            index--)
        {
            ulong firstWord = first[index];
            ulong secondWord = second[index];
            if (firstWord != secondWord)
                return firstWord < secondWord ? -1 : 1;
        }

        return 0;
    }

    private static int GetActiveMagnitudeLength(
        ReadOnlySpan<ulong> value)
    {
        int length = value.Length;
        while (length > 0 && value[length - 1] == 0UL)
            length--;
        return length;
    }

    private static void CopyMagnitude(
        ReadOnlySpan<ulong> source,
        Span<ulong> destination)
    {
        destination.Clear();
        source[..Math.Min(source.Length, destination.Length)]
            .CopyTo(destination);
    }

}
