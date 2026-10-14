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
    // its squared distance comparison needs fewer than 2252 bits for a
    // direction or 2312 bits for a full-domain segment-distance denominator.
    private const int TriangleSweepMagnitudeWords = 37;

    private static bool TryGetRationalSegmentSweepDistance(
        TriangleSweepPoint firstPoint,
        TriangleSweepPoint secondPoint,
        Vector2d start,
        in SweepTrajectory trajectory,
        Fixed64 maximumDistance,
        Fixed64 radius,
        out Fixed64 distance,
        out Vector3d localPoint,
        out Signed576 referenceYOffset)
    {
        SweepRationalPoint first = firstPoint.RationalPoint;
        SweepRationalPoint second = secondPoint.RationalPoint;
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
            localPoint = default;
            referenceYOffset = default;
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

        Signed192 directionX = trajectory.DirectionX;
        Signed192 directionZ = trajectory.DirectionZ;
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
        WideArithmetic.MultiplyMagnitudes(
            velocityCore,
            denominatorMagnitude,
            velocity);

        Span<ulong> edgeSquared =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        GetEdgeSquaredMagnitude(edgeX, edgeZ, edgeSquared);
        if (CompareRationalSegmentLineAt(
                startCross, startCrossSign, velocity, velocitySign,
                edgeSquared, denominatorMagnitude, radius,
                default, trajectory.TimeScale) <= 0)
        {
            distance = Fixed64.Zero;
            return TryGetExactRationalSegmentWitness(
                firstPoint, secondPoint, edgeX, edgeZ, edgeSquared,
                startX, startZ, startCross, velocity, trajectory, radius,
                atStart: true, out localPoint, out referenceYOffset);
        }

        // The expanded-line start test already returned for a zero cross.
        if (maximumDistance == Fixed64.Zero
            || velocitySign == 0
            || startCrossSign == velocitySign)
        {
            distance = default;
            localPoint = default;
            referenceYOffset = default;
            return false;
        }

        Span<ulong> startAtUnit =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> velocityAtMaximum =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> oneMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(trajectory.TimeScale, out oneMagnitude[2], out oneMagnitude[1], out oneMagnitude[0]);
        Span<ulong> maximumMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(
            Signed192.Signed(maximumDistance.m_rawValue),
            out maximumMagnitude[2],
            out maximumMagnitude[1],
            out maximumMagnitude[0]);
        WideArithmetic.MultiplyMagnitudes(startCross, oneMagnitude, startAtUnit);
        WideArithmetic.MultiplyMagnitudes(velocity, maximumMagnitude, velocityAtMaximum);
        bool closestBeyondMaximum =
            WideArithmetic.CompareMagnitudeEqualLength(startAtUnit, velocityAtMaximum) > 0;
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
                trajectory.TimeScale) > 0)
        {
            distance = default;
            localPoint = default;
            referenceYOffset = default;
            return false;
        }

        Span<ulong> values = stackalloc ulong[6 * TriangleSweepMagnitudeWords];
        Span<int> signs = stackalloc int[6];
        ContactQuadratic entry = ContactQuadratic.At(values, signs, 0, TriangleSweepMagnitudeWords);
        ContactQuadratic divisor = ContactQuadratic.At(values, signs, 1, TriangleSweepMagnitudeWords);
        ContactQuadratic work = ContactQuadratic.At(values, signs, 2, TriangleSweepMagnitudeWords);
        entry.Clear(); divisor.Clear(); work.Clear();
        // Entry in raw distance is (|cross|-R*D*sqrt(L))*TimeScale/|velocity|.
        startCross.CopyTo(entry.Rational); entry.Signs[0] = 1;
        Span<ulong> radiusMagnitude = stackalloc ulong[1] { (ulong)radius.m_rawValue };
        WideArithmetic.MultiplyMagnitudes(denominatorMagnitude, radiusMagnitude, entry.Radical);
        entry.Signs[1] = radius == Fixed64.Zero ? 0 : -1;
        velocity.CopyTo(divisor.Rational); divisor.Signs[0] = 1;
        ContactQuadratic.Scale(entry, Signed576.ExtendValue(Signed320.ExtendValue(trajectory.TimeScale)), work);
        work.CopyTo(entry);
        // Earlier admission bounds this positive entering root by the maximum distance.
        distance = ContactQuadratic.RoundRatio(entry, divisor, edgeSquared, 0L, maximumDistance.m_rawValue);

        if (TryGetExactRationalSegmentWitness(
                firstPoint, secondPoint, edgeX, edgeZ, edgeSquared,
                startX, startZ, startCross, velocity, trajectory, radius,
                atStart: false, out localPoint, out referenceYOffset))
            return true;

        distance = default;
        return false;
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
        WideArithmetic.MultiplyMagnitudes(
            startCross,
            timeDenominatorMagnitude,
            firstTerm);
        WideArithmetic.MultiplyMagnitudes(
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
        WideArithmetic.MultiplyMagnitudes(line, line, leftSquared);

        Span<ulong> radiusMagnitude = stackalloc ulong[1]
        {
            unchecked((ulong)radius.m_rawValue),
        };
        Span<ulong> right =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> scratch =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.MultiplyMagnitudes(edgeSquared, denominator, right);
        WideArithmetic.MultiplyMagnitudes(right, denominator, scratch);
        WideArithmetic.MultiplyMagnitudes(scratch, radiusMagnitude, right);
        WideArithmetic.MultiplyMagnitudes(right, radiusMagnitude, scratch);
        WideArithmetic.MultiplyMagnitudes(
            scratch,
            timeDenominatorMagnitude,
            right);
        WideArithmetic.MultiplyMagnitudes(
            right,
            timeDenominatorMagnitude,
            scratch);
        return WideArithmetic.CompareMagnitudeEqualLength(leftSquared, scratch);
    }

    private static bool TryGetExactRationalSegmentWitness(
        TriangleSweepPoint first,
        TriangleSweepPoint second,
        Signed832 edgeX,
        Signed832 edgeZ,
        ReadOnlySpan<ulong> edgeSquared,
        Signed576 startX,
        Signed576 startZ,
        ReadOnlySpan<ulong> startCross,
        ReadOnlySpan<ulong> velocityCross,
        in SweepTrajectory trajectory,
        Fixed64 radius,
        bool atStart,
        out Vector3d localPoint,
        out Signed576 referenceYOffset)
    {
        // Full clipped endpoints bound exact foot fields below 2508 bits;
        // multiplying by a 65-bit local difference stays below 2574 bits.
        const int words = 42;
        Span<ulong> values = stackalloc ulong[8 * words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic time = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic numerator = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic denominator = ContactQuadratic.At(values, signs, 2, words);
        ContactQuadratic work = ContactQuadratic.At(values, signs, 3, words);
        time.Clear(); numerator.Clear(); denominator.Clear(); work.Clear();
        Span<ulong> startDot = stackalloc ulong[TriangleSweepMagnitudeWords];
        GetDotMagnitude(edgeX, edgeZ, startX, startZ, startDot, out int startDotSign);
        Span<ulong> secondDenominator = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(second.RationalPoint.Denominator, secondDenominator);
        if (atStart)
        {
            startDot.CopyTo(numerator.Rational);
            numerator.Signs[0] = startDotSign;
            edgeSquared.CopyTo(denominator.Rational);
            denominator.Signs[0] = 1;
        }
        else
        {
            // The exact approaching root is (|cross|-R*D*sqrt(L))/|velocity|.
            // Retain this field when certifying and materializing its finite foot.
            startCross.CopyTo(time.Rational);
            time.Signs[0] = 1;
            Span<ulong> firstDenominator = stackalloc ulong[9];
            WideArithmetic.GetMagnitude(first.RationalPoint.Denominator, firstDenominator);
            Span<ulong> radiusMagnitude = stackalloc ulong[1] { (ulong)radius.m_rawValue };
            WideArithmetic.MultiplyMagnitudes(firstDenominator, radiusMagnitude, time.Radical);
            time.Signs[1] = radius == Fixed64.Zero ? 0 : -1;
            Span<ulong> velocityDot = stackalloc ulong[TriangleSweepMagnitudeWords];
            GetDotMagnitude(edgeX, edgeZ,
                WideArithmetic.MultiplySigned576(first.RationalPoint.Denominator, trajectory.DirectionX),
                WideArithmetic.MultiplySigned576(first.RationalPoint.Denominator, trajectory.DirectionZ),
                velocityDot, out int velocityDotSign);
            ContactQuadratic.Scale(time, velocityDot, velocityDotSign, numerator);
            WideArithmetic.MultiplyMagnitudes(startDot, velocityCross, work.Rational);
            work.Signs[0] = startDotSign;
            numerator.Add(work);
            WideArithmetic.MultiplyMagnitudes(edgeSquared, velocityCross, denominator.Rational);
            denominator.Signs[0] = 1;
        }
        ContactQuadratic.Scale(numerator, secondDenominator, 1, work);
        work.CopyTo(numerator);
        numerator.CopyTo(time);
        time.Add(denominator, -1);
        if (numerator.Sign(edgeSquared) < 0 || time.Sign(edgeSquared) > 0)
        {
            localPoint = default;
            referenceYOffset = default;
            return false;
        }

        localPoint = new Vector3d(
            RoundRationalSegmentCoordinate(first.LocalPoint.X, second.LocalPoint.X, numerator, denominator, edgeSquared, work, time),
            RoundRationalSegmentCoordinate(first.LocalPoint.Y, second.LocalPoint.Y, numerator, denominator, edgeSquared, work, time),
            RoundRationalSegmentCoordinate(first.LocalPoint.Z, second.LocalPoint.Z, numerator, denominator, edgeSquared, work, time));
        ContactQuadratic.Scale(numerator, Signed576.ExtendValue(Signed320.ExtendValue(SweepRawScale)), work);
        Fixed64 parameter = ContactQuadratic.RoundRatio(work, denominator, edgeSquared, 0L, FixedMath.ONE_L);
        referenceYOffset = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(first.ReferenceYOffset, Signed320.ExtendValue(Signed192.Raw(Fixed64.One - parameter))),
            WideArithmetic.MultiplySigned320(second.ReferenceYOffset, Signed320.ExtendValue(Signed192.Raw(parameter))));
        return true;
    }

    private static Fixed64 RoundRationalSegmentCoordinate(
        Fixed64 first,
        Fixed64 second,
        ContactQuadratic numerator,
        ContactQuadratic denominator,
        ReadOnlySpan<ulong> root,
        ContactQuadratic work,
        ContactQuadratic scratch)
    {
        Signed192 delta = WideArithmetic.SubtractSigned192(Signed192.Raw(second), Signed192.Raw(first));
        ContactQuadratic.Scale(numerator, Signed576.ExtendValue(Signed320.ExtendValue(delta)), work);
        ContactQuadratic.Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(first))), scratch);
        work.Add(scratch);
        return ContactQuadratic.RoundRatio(work, denominator, root,
            Math.Min(first.m_rawValue, second.m_rawValue), Math.Max(first.m_rawValue, second.m_rawValue));
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
        WideArithmetic.MultiplyMagnitudes(x, x, xSquared);
        WideArithmetic.MultiplyMagnitudes(z, z, zSquared);
        WideArithmetic.AddEqualMagnitudes(xSquared, zSquared, result);
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
        WideArithmetic.MultiplyMagnitudes(firstX, secondZ, firstTerm);
        WideArithmetic.MultiplyMagnitudes(firstZ, secondX, secondTerm);
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
        WideArithmetic.MultiplyMagnitudes(firstXMag, secondXMag, firstTerm);
        WideArithmetic.MultiplyMagnitudes(firstZMag, secondZMag, secondTerm);
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
            WideArithmetic.AddEqualMagnitudes(first, second, result);
            sign = firstSign;
            return;
        }

        int comparison = WideArithmetic.CompareMagnitudeEqualLength(first, second);
        if (comparison == 0)
        {
            result.Clear();
            sign = 0;
            return;
        }
        if (comparison > 0)
        {
            WideArithmetic.SubtractEqualMagnitudes(first, second, result);
            sign = firstSign;
            return;
        }

        WideArithmetic.SubtractEqualMagnitudes(second, first, result);
        sign = secondSign;
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
