//=======================================================================
// WideOrientedBox.Box.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Implements oriented bounding box (OBB) vs OBB collision detection using the
/// separating axis theorem (SAT), computing contact anchor points on overlap.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct BoxPenetration
    {
        internal readonly WideAxis3 Axis;
        internal readonly bool Negate;
        internal readonly Signed576 Overlap;
        internal readonly Signed576 SquaredAxisLength;

        internal BoxPenetration(
            WideAxis3 axis,
            bool negate,
            Signed576 overlap,
            Signed576 squaredAxisLength)
        {
            Axis = axis;
            Negate = negate;
            Overlap = overlap;
            SquaredAxisLength = squaredAxisLength;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }

    #endregion

    internal static bool TryGetContact(
        Vector3d firstCenter,
        FixedQuaternion firstOrientation,
        Vector3d firstHalfExtents,
        Vector3d secondCenter,
        FixedQuaternion secondOrientation,
        Vector3d secondHalfExtents,
        out FixedContactAnchors contact)
    {
        RationalBasis firstBasis = new(firstOrientation);
        RationalBasis secondBasis = new(secondOrientation);
        Signed320 commonDenominator = WideArithmetic.MultiplySigned192(
            firstBasis.Denominator,
            secondBasis.Denominator);
        Span<WideAxis3> firstAxes = stackalloc WideAxis3[3]
        {
            GetBasisAxis(firstBasis, 0),
            GetBasisAxis(firstBasis, 1),
            GetBasisAxis(firstBasis, 2),
        };
        Span<WideAxis3> secondAxes = stackalloc WideAxis3[3]
        {
            GetBasisAxis(secondBasis, 0),
            GetBasisAxis(secondBasis, 1),
            GetBasisAxis(secondBasis, 2),
        };
        var best = default(BoxPenetration);
        for (int index = 0; index < firstAxes.Length; index++)
        {
            if (!TryKeepBoxAxis(
                    firstAxes[index],
                    firstCenter,
                    firstHalfExtents,
                    firstBasis,
                    secondCenter,
                    secondHalfExtents,
                    secondBasis,
                    commonDenominator,
                    ref best)
                || !TryKeepBoxAxis(
                    secondAxes[index],
                    firstCenter,
                    firstHalfExtents,
                    firstBasis,
                    secondCenter,
                    secondHalfExtents,
                    secondBasis,
                    commonDenominator,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        for (int firstIndex = 0; firstIndex < firstAxes.Length; firstIndex++)
        {
            for (int secondIndex = 0; secondIndex < secondAxes.Length; secondIndex++)
            {
                if (!TryKeepBoxAxis(
                        Cross(firstAxes[firstIndex], secondAxes[secondIndex]),
                        firstCenter,
                        firstHalfExtents,
                        firstBasis,
                        secondCenter,
                        secondHalfExtents,
                        secondBasis,
                        commonDenominator,
                        ref best))
                {
                    contact = default;
                    return false;
                }
            }
        }

        Fixed64 depth =
            WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
                best.Overlap,
                best.SquaredAxisLength,
                commonDenominator,
                out bool depthIsClamped);
        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        Vector3d normal = WideGeometry.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        var secondOrigin = new FixedPointAnchor(
            secondCenter,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        Vector3d firstLocalPoint = GetMatchedBoxSupportLocalPoint(
            firstBasis,
            firstHalfExtents,
            firstCenter,
            firstOrientation,
            secondOrigin,
            orientedAxis);
        var firstAnchor = new FixedPointAnchor(
            firstCenter,
            firstOrientation,
            firstLocalPoint);
        Vector3d secondLocalPoint = GetMatchedBoxSupportLocalPoint(
            secondBasis,
            secondHalfExtents,
            secondCenter,
            secondOrientation,
            firstAnchor,
            -orientedAxis);
        contact = new FixedContactAnchors(
            firstAnchor,
            new FixedPointAnchor(
                secondCenter,
                secondOrientation,
                secondLocalPoint),
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryKeepBoxAxis(
        WideAxis3 axis,
        Vector3d firstCenter,
        Vector3d firstHalfExtents,
        RationalBasis firstBasis,
        Vector3d secondCenter,
        Vector3d secondHalfExtents,
        RationalBasis secondBasis,
        Signed320 commonDenominator,
        ref BoxPenetration best)
    {
        if (axis.IsZero)
            return true;

        Signed576 firstRadius = GetBoxProjectionRadiusNumerator(
            axis,
            firstHalfExtents,
            firstBasis);
        Signed576 secondRadius = GetBoxProjectionRadiusNumerator(
            axis,
            secondHalfExtents,
            secondBasis);
        Signed576 centerProjection = GetDifferenceProjection(
            secondCenter,
            firstCenter,
            axis);
        bool negate = centerProjection.Sign < 0;
        Signed576 overlap = WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    firstRadius,
                    secondBasis.Denominator),
                WideArithmetic.MultiplySigned576(
                    secondRadius,
                    firstBasis.Denominator)),
            WideArithmetic.MultiplySigned576(
                GetMagnitude(centerProjection),
                Signed192.NarrowValue(commonDenominator)));
        if (overlap.Sign < 0)
            return false;

        Signed576 squaredAxisLength = GetSquaredLength(axis);
        bool shouldReplace = !best.HasValue
            || CompareBoxDepth(
                overlap,
                squaredAxisLength,
                best.Overlap,
                best.SquaredAxisLength) < 0;
        if (shouldReplace)
        {
            best = new BoxPenetration(
                axis,
                negate,
                overlap,
                squaredAxisLength);
        }
        return true;
    }

    private static int CompareBoxDepth(
        Signed576 candidateOverlap,
        Signed576 candidateSquaredAxisLength,
        Signed576 currentOverlap,
        Signed576 currentSquaredAxisLength)
    {
        Span<ulong> candidate = stackalloc ulong[9];
        Span<ulong> current = stackalloc ulong[9];
        Span<ulong> candidateAxis = stackalloc ulong[9];
        Span<ulong> currentAxis = stackalloc ulong[9];
        Span<ulong> candidateSquared = stackalloc ulong[18];
        Span<ulong> currentSquared = stackalloc ulong[18];
        Span<ulong> candidateScaled =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> currentScaled =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.GetMagnitude(candidateOverlap, candidate);
        WideArithmetic.GetMagnitude(currentOverlap, current);
        WideArithmetic.GetMagnitude(
            candidateSquaredAxisLength,
            candidateAxis);
        WideArithmetic.GetMagnitude(
            currentSquaredAxisLength,
            currentAxis);
        MultiplyMagnitudes(candidate, candidate, candidateSquared);
        MultiplyMagnitudes(current, current, currentSquared);
        MultiplyMagnitudes(
            candidateSquared,
            currentAxis,
            candidateScaled);
        MultiplyMagnitudes(
            currentSquared,
            candidateAxis,
            currentScaled);
        return CompareMagnitudes(candidateScaled, currentScaled);
    }

    private static WideAxis3 GetBasisAxis(RationalBasis basis, int index) =>
        index switch
        {
            0 => new WideAxis3(
                Signed320.ExtendValue(basis.Xx),
                Signed320.ExtendValue(basis.Xy),
                Signed320.ExtendValue(basis.Xz)),
            1 => new WideAxis3(
                Signed320.ExtendValue(basis.Yx),
                Signed320.ExtendValue(basis.Yy),
                Signed320.ExtendValue(basis.Yz)),
            _ => new WideAxis3(
                Signed320.ExtendValue(basis.Zx),
                Signed320.ExtendValue(basis.Zy),
                Signed320.ExtendValue(basis.Zz)),
        };

    private static WideAxis3 Cross(WideAxis3 left, WideAxis3 right) =>
        // Each rational quaternion-basis component uses at most 65 significant
        // bits, so a difference of two component products fits in Signed320.
        new(Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(left.Y, right.Z),
                WideArithmetic.MultiplySigned320(left.Z, right.Y))),
            Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(left.Z, right.X),
                WideArithmetic.MultiplySigned320(left.X, right.Z))),
            Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(left.X, right.Y),
                WideArithmetic.MultiplySigned320(left.Y, right.X))));
}
