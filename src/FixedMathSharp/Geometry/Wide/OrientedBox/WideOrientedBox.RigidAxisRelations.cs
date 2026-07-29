//=======================================================================
// WideOrientedBox.RigidAxisRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides high-precision (wide arithmetic) computations for rigid axis relations,
/// including capsule overlap tests based on rotated local axes and segment distances.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct RigidSegmentAxis
    {
        internal readonly Signed320 X;
        internal readonly Signed320 Y;
        internal readonly Signed320 Z;
        internal readonly Signed320 Denominator;

        internal RigidSegmentAxis(
            Signed192 x,
            Signed192 y,
            Signed192 z,
            Signed192 rotationDenominator,
            Fixed64 length)
        {
            Signed192 rawLength = Signed192.Raw(length);
            X = WideArithmetic.MultiplySigned192(x, rawLength);
            Y = WideArithmetic.MultiplySigned192(y, rawLength);
            Z = WideArithmetic.MultiplySigned192(z, rawLength);
            Denominator = WideArithmetic.MultiplySigned192(
                rotationDenominator,
                Signed192.One);
        }
    }

    #endregion

    internal static bool DoCenteredRigidCapsulesOverlap(
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Vector3d firstLocalAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Vector3d secondLocalAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius)
    {
        GetRotatedLocalAxisNumerators(
            firstRotation,
            firstLocalAxis,
            out Signed192 firstAxisX,
            out Signed192 firstAxisY,
            out Signed192 firstAxisZ,
            out Signed192 firstRotationDenominator);
        GetRotatedLocalAxisNumerators(
            secondRotation,
            secondLocalAxis,
            out Signed192 secondAxisX,
            out Signed192 secondAxisY,
            out Signed192 secondAxisZ,
            out Signed192 secondRotationDenominator);

        var firstAxis = new RigidSegmentAxis(
            firstAxisX,
            firstAxisY,
            firstAxisZ,
            firstRotationDenominator,
            firstLength);
        var secondAxis = new RigidSegmentAxis(
            secondAxisX,
            secondAxisY,
            secondAxisZ,
            secondRotationDenominator,
            secondLength);
        Signed576 relationDenominator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                firstAxis.Denominator,
                secondAxis.Denominator),
            WideArithmetic.MultiplySigned320(
                firstAxis.Denominator,
                secondAxis.Denominator));
        GetRigidSegmentLowerDifference(
            firstCenter,
            firstAxis,
            secondCenter,
            secondAxis,
            relationDenominator,
            out Signed576 relationX,
            out Signed576 relationY,
            out Signed576 relationZ);

        Signed576 firstSquared = Dot(firstAxis, firstAxis);
        Signed576 directionsDot = Dot(firstAxis, secondAxis);
        Signed576 secondSquared = Dot(secondAxis, secondAxis);
        Signed704 firstDotDifference = Dot(
            firstAxis,
            relationX,
            relationY,
            relationZ);
        Signed704 secondDotDifference = Dot(
            secondAxis,
            relationX,
            relationY,
            relationZ);

        Span<ulong> determinant = stackalloc ulong[TriangleSweepMagnitudeWords];
        GetProductDifference(
            firstSquared,
            secondSquared,
            directionsDot,
            directionsDot,
            determinant,
            out int determinantSign);

        Span<ulong> firstNumerator = stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> firstDenominator = stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> secondNumerator = stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> secondDenominator = stackalloc ulong[TriangleSweepMagnitudeWords];
        int firstNumeratorSign;
        int secondNumeratorSign;
        int firstFeature;
        int secondFeature;
        if (determinantSign > 0)
        {
            Span<ulong> firstCore = stackalloc ulong[TriangleSweepMagnitudeWords];
            GetProductDifference(
                directionsDot,
                secondDotDifference,
                secondSquared,
                firstDotDifference,
                firstCore,
                out int firstCoreSign);
            SetRigidSegmentRatio(
                firstCore,
                firstAxis.Denominator,
                determinant,
                relationDenominator,
                firstNumerator,
                firstDenominator);
            firstNumeratorSign = firstCoreSign;
            firstFeature = ClampUnitRatio(
                firstNumerator,
                firstDenominator,
                firstNumeratorSign);

            if (firstFeature == RigidSegmentLower)
            {
                SetRigidSegmentProjectionRatio(
                    secondDotDifference,
                    secondAxis.Denominator,
                    secondSquared,
                    relationDenominator,
                    secondNumerator,
                    secondDenominator);
                secondNumeratorSign = secondDotDifference.Sign;
            }
            else if (firstFeature == RigidSegmentUpper)
            {
                Signed704 upperProjection = WideArithmetic.AddSigned704(
                    secondDotDifference,
                    DoubleProduct(
                        directionsDot,
                        secondAxis.Denominator));
                SetRigidSegmentProjectionRatio(
                    upperProjection,
                    secondAxis.Denominator,
                    secondSquared,
                    relationDenominator,
                    secondNumerator,
                    secondDenominator);
                secondNumeratorSign = upperProjection.Sign;
            }
            else
            {
                Span<ulong> secondCore = stackalloc ulong[TriangleSweepMagnitudeWords];
                GetProductDifference(
                    firstSquared,
                    secondDotDifference,
                    directionsDot,
                    firstDotDifference,
                    secondCore,
                    out int secondCoreSign);
                SetRigidSegmentRatio(
                    secondCore,
                    secondAxis.Denominator,
                    determinant,
                    relationDenominator,
                    secondNumerator,
                    secondDenominator);
                secondNumeratorSign = secondCoreSign;
            }
        }
        else
        {
            firstNumerator.Clear();
            firstDenominator.Clear();
            firstDenominator[0] = 1UL;
            firstNumeratorSign = 0;
            firstFeature = RigidSegmentLower;
            SetRigidSegmentProjectionRatio(
                secondDotDifference,
                secondAxis.Denominator,
                secondSquared,
                relationDenominator,
                secondNumerator,
                secondDenominator);
            secondNumeratorSign = secondDotDifference.Sign;
        }

        secondFeature = ClampUnitRatio(
            secondNumerator,
            secondDenominator,
            secondNumeratorSign);
        if (secondFeature == RigidSegmentLower)
        {
            SetRigidSegmentProjectionRatio(
                WideArithmetic.SubtractSigned704(
                    default,
                    firstDotDifference),
                firstAxis.Denominator,
                firstSquared,
                relationDenominator,
                firstNumerator,
                firstDenominator);
            firstNumeratorSign = -firstDotDifference.Sign;
            firstFeature = ClampUnitRatio(
                firstNumerator,
                firstDenominator,
                firstNumeratorSign);
        }
        else if (secondFeature == RigidSegmentUpper)
        {
            Signed704 upperProjection = WideArithmetic.SubtractSigned704(
                DoubleProduct(
                    directionsDot,
                    firstAxis.Denominator),
                firstDotDifference);
            SetRigidSegmentProjectionRatio(
                upperProjection,
                firstAxis.Denominator,
                firstSquared,
                relationDenominator,
                firstNumerator,
                firstDenominator);
            firstNumeratorSign = upperProjection.Sign;
            firstFeature = ClampUnitRatio(
                firstNumerator,
                firstDenominator,
                firstNumeratorSign);
        }

        if (firstFeature == RigidSegmentUpper)
        {
            AddRigidSegmentDisplacement(
                ref relationX,
                firstAxis.X,
                secondAxis.Denominator);
            AddRigidSegmentDisplacement(
                ref relationY,
                firstAxis.Y,
                secondAxis.Denominator);
            AddRigidSegmentDisplacement(
                ref relationZ,
                firstAxis.Z,
                secondAxis.Denominator);
        }
        if (secondFeature == RigidSegmentUpper)
        {
            SubtractRigidSegmentDisplacement(
                ref relationX,
                secondAxis.X,
                firstAxis.Denominator);
            SubtractRigidSegmentDisplacement(
                ref relationY,
                secondAxis.Y,
                firstAxis.Denominator);
            SubtractRigidSegmentDisplacement(
                ref relationZ,
                secondAxis.Z,
                firstAxis.Denominator);
        }

        Signed192 combinedRadius = WideArithmetic.AddSigned192(
            Signed192.Raw(firstRadius),
            Signed192.Raw(secondRadius));
        if (firstFeature == RigidSegmentInterior
            && secondFeature == RigidSegmentInterior)
        {
            return IsRigidLinePairWithinRadius(
                relationX,
                relationY,
                relationZ,
                firstSquared,
                directionsDot,
                secondSquared,
                firstDotDifference,
                secondDotDifference,
                determinant,
                relationDenominator,
                combinedRadius);
        }
        if (firstFeature == RigidSegmentInterior)
        {
            Signed704 adjustedDot = Dot(
                firstAxis,
                relationX,
                relationY,
                relationZ);
            return IsRigidPointLineWithinRadius(
                relationX,
                relationY,
                relationZ,
                firstSquared,
                adjustedDot,
                relationDenominator,
                combinedRadius);
        }
        if (secondFeature == RigidSegmentInterior)
        {
            Signed704 adjustedDot = Dot(
                secondAxis,
                relationX,
                relationY,
                relationZ);
            return IsRigidPointLineWithinRadius(
                relationX,
                relationY,
                relationZ,
                secondSquared,
                adjustedDot,
                relationDenominator,
                combinedRadius);
        }

        return IsRigidPointWithinRadius(
            relationX,
            relationY,
            relationZ,
            relationDenominator,
            combinedRadius);
    }

    private const int RigidSegmentLower = -1;
    private const int RigidSegmentInterior = 0;
    private const int RigidSegmentUpper = 1;

    private static int ClampUnitRatio(
        Span<ulong> numerator,
        Span<ulong> denominator,
        int numeratorSign)
    {
        if (numeratorSign <= 0)
        {
            numerator.Clear();
            denominator.Clear();
            denominator[0] = 1UL;
            return RigidSegmentLower;
        }

        int comparison = WideArithmetic.CompareMagnitudeEqualLength(numerator, denominator);
        if (comparison < 0)
            return RigidSegmentInterior;

        denominator.CopyTo(numerator);
        return RigidSegmentUpper;
    }

    private static void SetRigidSegmentRatio(
        ReadOnlySpan<ulong> core,
        Signed320 axisDenominator,
        ReadOnlySpan<ulong> determinant,
        Signed576 relationDenominator,
        Span<ulong> numerator,
        Span<ulong> denominator)
    {
        Span<ulong> axisDenominatorMagnitude = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(
            axisDenominator,
            out axisDenominatorMagnitude[4],
            out axisDenominatorMagnitude[3],
            out axisDenominatorMagnitude[2],
            out axisDenominatorMagnitude[1],
            out axisDenominatorMagnitude[0]);
        WideArithmetic.MultiplyMagnitudes(core, axisDenominatorMagnitude, numerator);

        Span<ulong> relationDenominatorMagnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(
            relationDenominator,
            relationDenominatorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            determinant,
            relationDenominatorMagnitude,
            denominator);
    }

    private static void SetRigidSegmentProjectionRatio(
        Signed704 projection,
        Signed320 axisDenominator,
        Signed576 axisSquared,
        Signed576 relationDenominator,
        Span<ulong> numerator,
        Span<ulong> denominator)
    {
        Span<ulong> projectionMagnitude = stackalloc ulong[11];
        Span<ulong> axisDenominatorMagnitude = stackalloc ulong[5];
        Span<ulong> axisSquaredMagnitude = stackalloc ulong[9];
        Span<ulong> relationDenominatorMagnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(projection, projectionMagnitude);
        WideArithmetic.GetMagnitude(
            axisDenominator,
            out axisDenominatorMagnitude[4],
            out axisDenominatorMagnitude[3],
            out axisDenominatorMagnitude[2],
            out axisDenominatorMagnitude[1],
            out axisDenominatorMagnitude[0]);
        WideArithmetic.GetMagnitude(axisSquared, axisSquaredMagnitude);
        WideArithmetic.GetMagnitude(
            relationDenominator,
            relationDenominatorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            projectionMagnitude,
            axisDenominatorMagnitude,
            numerator);
        WideArithmetic.MultiplyMagnitudes(
            axisSquaredMagnitude,
            relationDenominatorMagnitude,
            denominator);
    }

    private static bool IsRigidLinePairWithinRadius(
        Signed576 relationX,
        Signed576 relationY,
        Signed576 relationZ,
        Signed576 firstSquared,
        Signed576 directionsDot,
        Signed576 secondSquared,
        Signed704 firstDotDifference,
        Signed704 secondDotDifference,
        ReadOnlySpan<ulong> determinant,
        Signed576 relationDenominator,
        Signed192 radius)
    {
        Span<ulong> relationSquared = stackalloc ulong[13];
        GetSquaredMagnitude(
            relationX,
            relationY,
            relationZ,
            relationSquared);
        Span<ulong> relationTerm = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.MultiplyMagnitudes(
            relationSquared,
            determinant,
            relationTerm);

        Span<ulong> firstCorrection = stackalloc ulong[TriangleSweepMagnitudeWords];
        GetTripleProductMagnitude(
            secondSquared,
            firstDotDifference,
            firstDotDifference,
            firstCorrection);
        Span<ulong> mixedCorrection = stackalloc ulong[TriangleSweepMagnitudeWords];
        GetTripleProductMagnitude(
            directionsDot,
            firstDotDifference,
            secondDotDifference,
            mixedCorrection);
        DoubleMagnitude(mixedCorrection);
        Span<ulong> secondCorrection = stackalloc ulong[TriangleSweepMagnitudeWords];
        GetTripleProductMagnitude(
            firstSquared,
            secondDotDifference,
            secondDotDifference,
            secondCorrection);

        Span<ulong> correction = stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> correctionScratch = stackalloc ulong[TriangleSweepMagnitudeWords];
        CombineSignedMagnitudes(
            firstCorrection,
            1,
            mixedCorrection,
            -(directionsDot.Sign
                * firstDotDifference.Sign
                * secondDotDifference.Sign),
            correctionScratch,
            out int correctionSign);
        CombineSignedMagnitudes(
            correctionScratch,
            correctionSign,
            secondCorrection,
            1,
            correction,
            out _);

        Span<ulong> left = stackalloc ulong[TriangleSweepMagnitudeWords];
        // An interior/interior closest pair has a strictly positive projected
        // correction. A zero parameter is classified as a cap before reaching
        // this branch.
        WideArithmetic.SubtractEqualMagnitudes(relationTerm, correction, left);

        Span<ulong> right = stackalloc ulong[TriangleSweepMagnitudeWords];
        GetRadiusThreshold(
            radius,
            relationDenominator,
            determinant,
            right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right) <= 0;
    }

    private static bool IsRigidPointLineWithinRadius(
        Signed576 relationX,
        Signed576 relationY,
        Signed576 relationZ,
        Signed576 axisSquared,
        Signed704 axisDotDifference,
        Signed576 relationDenominator,
        Signed192 radius)
    {
        Span<ulong> relationSquared = stackalloc ulong[13];
        GetSquaredMagnitude(
            relationX,
            relationY,
            relationZ,
            relationSquared);
        Span<ulong> axisSquaredMagnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(axisSquared, axisSquaredMagnitude);
        Span<ulong> relationTerm = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.MultiplyMagnitudes(
            relationSquared,
            axisSquaredMagnitude,
            relationTerm);
        Span<ulong> dotMagnitude = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(axisDotDifference, dotMagnitude);
        Span<ulong> dotSquared = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.MultiplyMagnitudes(dotMagnitude, dotMagnitude, dotSquared);
        Span<ulong> left = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.SubtractEqualMagnitudes(relationTerm, dotSquared, left);

        Span<ulong> right = stackalloc ulong[TriangleSweepMagnitudeWords];
        GetRadiusThreshold(
            radius,
            relationDenominator,
            axisSquaredMagnitude,
            right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right) <= 0;
    }

    private static bool IsRigidPointWithinRadius(
        Signed576 relationX,
        Signed576 relationY,
        Signed576 relationZ,
        Signed576 relationDenominator,
        Signed192 radius)
    {
        Span<ulong> left = stackalloc ulong[TriangleSweepMagnitudeWords];
        left.Clear();
        GetSquaredMagnitude(
            relationX,
            relationY,
            relationZ,
            left);
        Span<ulong> right = stackalloc ulong[TriangleSweepMagnitudeWords];
        GetRadiusThreshold(
            radius,
            relationDenominator,
            ReadOnlySpan<ulong>.Empty,
            right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right) <= 0;
    }

    private static void GetRadiusThreshold(
        Signed192 radius,
        Signed576 relationDenominator,
        ReadOnlySpan<ulong> factor,
        Span<ulong> result)
    {
        Span<ulong> radiusMagnitude = stackalloc ulong[3];
        Span<ulong> denominatorMagnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(
            radius,
            out radiusMagnitude[2],
            out radiusMagnitude[1],
            out radiusMagnitude[0]);
        WideArithmetic.GetMagnitude(
            relationDenominator,
            denominatorMagnitude);
        Span<ulong> radiusSquared = stackalloc ulong[6];
        Span<ulong> denominatorSquared = stackalloc ulong[18];
        WideArithmetic.MultiplyMagnitudes(radiusMagnitude, radiusMagnitude, radiusSquared);
        WideArithmetic.MultiplyMagnitudes(
            denominatorMagnitude,
            denominatorMagnitude,
            denominatorSquared);
        Span<ulong> baseThreshold = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.MultiplyMagnitudes(
            radiusSquared,
            denominatorSquared,
            baseThreshold);
        if (factor.IsEmpty)
        {
            baseThreshold.CopyTo(result);
            return;
        }

        WideArithmetic.MultiplyMagnitudes(baseThreshold, factor, result);
    }

    private static void GetSquaredMagnitude(
        Signed576 x,
        Signed576 y,
        Signed576 z,
        Span<ulong> result)
    {
        Signed832 squared = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(x, x),
                WideArithmetic.MultiplySigned576ToSigned832(y, y)),
            WideArithmetic.MultiplySigned576ToSigned832(z, z));
        WideArithmetic.GetMagnitude(squared, result);
    }

    private static void GetTripleProductMagnitude(
        Signed576 first,
        Signed704 second,
        Signed704 third,
        Span<ulong> result)
    {
        Span<ulong> firstMagnitude = stackalloc ulong[9];
        Span<ulong> secondMagnitude = stackalloc ulong[11];
        Span<ulong> thirdMagnitude = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(first, firstMagnitude);
        WideArithmetic.GetMagnitude(second, secondMagnitude);
        WideArithmetic.GetMagnitude(third, thirdMagnitude);
        Span<ulong> firstProduct = stackalloc ulong[20];
        WideArithmetic.MultiplyMagnitudes(
            firstMagnitude,
            secondMagnitude,
            firstProduct);
        WideArithmetic.MultiplyMagnitudes(firstProduct, thirdMagnitude, result);
    }

    private static void GetProductDifference(
        Signed576 firstLeft,
        Signed576 firstRight,
        Signed576 secondLeft,
        Signed576 secondRight,
        Span<ulong> result,
        out int sign)
    {
        Span<ulong> firstLeftMagnitude = stackalloc ulong[9];
        Span<ulong> firstRightMagnitude = stackalloc ulong[9];
        Span<ulong> secondLeftMagnitude = stackalloc ulong[9];
        Span<ulong> secondRightMagnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(firstLeft, firstLeftMagnitude);
        WideArithmetic.GetMagnitude(firstRight, firstRightMagnitude);
        WideArithmetic.GetMagnitude(secondLeft, secondLeftMagnitude);
        WideArithmetic.GetMagnitude(secondRight, secondRightMagnitude);
        Span<ulong> first = stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> second = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.MultiplyMagnitudes(
            firstLeftMagnitude,
            firstRightMagnitude,
            first);
        WideArithmetic.MultiplyMagnitudes(
            secondLeftMagnitude,
            secondRightMagnitude,
            second);
        CombineSignedMagnitudes(
            first,
            firstLeft.Sign * firstRight.Sign,
            second,
            -(secondLeft.Sign * secondRight.Sign),
            result,
            out sign);
    }

    private static void GetProductDifference(
        Signed576 firstLeft,
        Signed704 firstRight,
        Signed576 secondLeft,
        Signed704 secondRight,
        Span<ulong> result,
        out int sign)
    {
        Span<ulong> firstLeftMagnitude = stackalloc ulong[9];
        Span<ulong> firstRightMagnitude = stackalloc ulong[11];
        Span<ulong> secondLeftMagnitude = stackalloc ulong[9];
        Span<ulong> secondRightMagnitude = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(firstLeft, firstLeftMagnitude);
        WideArithmetic.GetMagnitude(firstRight, firstRightMagnitude);
        WideArithmetic.GetMagnitude(secondLeft, secondLeftMagnitude);
        WideArithmetic.GetMagnitude(secondRight, secondRightMagnitude);
        Span<ulong> first = stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> second = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.MultiplyMagnitudes(
            firstLeftMagnitude,
            firstRightMagnitude,
            first);
        WideArithmetic.MultiplyMagnitudes(
            secondLeftMagnitude,
            secondRightMagnitude,
            second);
        CombineSignedMagnitudes(
            first,
            firstLeft.Sign * firstRight.Sign,
            second,
            -(secondLeft.Sign * secondRight.Sign),
            result,
            out sign);
    }

    private static void DoubleMagnitude(Span<ulong> value)
    {
        ulong carry = 0UL;
        for (int index = 0; index < value.Length; index++)
        {
            ulong nextCarry = value[index] >> 63;
            value[index] = (value[index] << 1) | carry;
            carry = nextCarry;
        }
    }

    private static Signed704 DoubleProduct(
        Signed576 value,
        Signed320 factor)
    {
        Signed704 product =
            WideArithmetic.MultiplySigned576ToSigned704(value, factor);
        return WideArithmetic.AddSigned704(product, product);
    }

    private static void GetRigidSegmentLowerDifference(
        Vector3d firstCenter,
        RigidSegmentAxis firstAxis,
        Vector3d secondCenter,
        RigidSegmentAxis secondAxis,
        Signed576 relationDenominator,
        out Signed576 x,
        out Signed576 y,
        out Signed576 z)
    {
        x = GetRigidSegmentLowerDifference(
            firstCenter.X,
            firstAxis.X,
            firstAxis.Denominator,
            secondCenter.X,
            secondAxis.X,
            secondAxis.Denominator,
            relationDenominator);
        y = GetRigidSegmentLowerDifference(
            firstCenter.Y,
            firstAxis.Y,
            firstAxis.Denominator,
            secondCenter.Y,
            secondAxis.Y,
            secondAxis.Denominator,
            relationDenominator);
        z = GetRigidSegmentLowerDifference(
            firstCenter.Z,
            firstAxis.Z,
            firstAxis.Denominator,
            secondCenter.Z,
            secondAxis.Z,
            secondAxis.Denominator,
            relationDenominator);
    }

    private static Signed576 GetRigidSegmentLowerDifference(
        Fixed64 firstCenter,
        Signed320 firstAxis,
        Signed320 firstDenominator,
        Fixed64 secondCenter,
        Signed320 secondAxis,
        Signed320 secondDenominator,
        Signed576 relationDenominator)
    {
        Signed576 centerTerm = WideArithmetic.MultiplySigned576(
            relationDenominator,
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(firstCenter),
                Signed192.Raw(secondCenter)));
        Signed576 firstEndpoint = WideArithmetic.MultiplySigned320(
            firstAxis,
            secondDenominator);
        Signed576 secondEndpoint = WideArithmetic.MultiplySigned320(
            secondAxis,
            firstDenominator);
        return WideArithmetic.AddSigned576(
            WideArithmetic.SubtractSigned576(
                centerTerm,
                firstEndpoint),
            secondEndpoint);
    }

    private static void AddRigidSegmentDisplacement(
        ref Signed576 relation,
        Signed320 axis,
        Signed320 otherDenominator)
    {
        Signed576 displacement = WideArithmetic.MultiplySigned320(
            axis,
            otherDenominator);
        relation = WideArithmetic.AddSigned576(
            relation,
            WideArithmetic.AddSigned576(displacement, displacement));
    }

    private static void SubtractRigidSegmentDisplacement(
        ref Signed576 relation,
        Signed320 axis,
        Signed320 otherDenominator)
    {
        Signed576 displacement = WideArithmetic.MultiplySigned320(
            axis,
            otherDenominator);
        relation = WideArithmetic.SubtractSigned576(
            relation,
            WideArithmetic.AddSigned576(displacement, displacement));
    }

    private static Signed576 Dot(
        RigidSegmentAxis first,
        RigidSegmentAxis second) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(first.X, second.X),
                WideArithmetic.MultiplySigned320(first.Y, second.Y)),
            WideArithmetic.MultiplySigned320(first.Z, second.Z));

    private static Signed704 Dot(
        RigidSegmentAxis axis,
        Signed576 x,
        Signed576 y,
        Signed576 z) =>
        WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(x, axis.X),
                WideArithmetic.MultiplySigned576ToSigned704(y, axis.Y)),
            WideArithmetic.MultiplySigned576ToSigned704(z, axis.Z));
}
