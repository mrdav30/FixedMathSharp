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
        internal readonly int FirstAxisIndex;
        internal readonly int SecondAxisIndex;
        internal readonly bool Negate;
        internal readonly Signed576 Overlap;
        internal readonly Signed576 SquaredAxisLength;

        internal BoxPenetration(
            int firstAxisIndex,
            int secondAxisIndex,
            bool negate,
            Signed576 overlap,
            Signed576 squaredAxisLength)
        {
            FirstAxisIndex = firstAxisIndex;
            SecondAxisIndex = secondAxisIndex;
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
        WideRationalBasis3d firstBasis = new(firstOrientation);
        WideRationalBasis3d secondBasis = new(secondOrientation);
        WideRationalBasis3d relativeBasis =
            WideRationalBasis3d.CreateRelative(firstBasis, secondBasis);
        Signed320 commonDenominator =
            Signed320.ExtendValue(relativeBasis.Denominator);
        Span<Signed192> relative = stackalloc Signed192[9]
        {
            relativeBasis.Xx,
            relativeBasis.Xy,
            relativeBasis.Xz,
            relativeBasis.Yx,
            relativeBasis.Yy,
            relativeBasis.Yz,
            relativeBasis.Zx,
            relativeBasis.Zy,
            relativeBasis.Zz,
        };
        Span<Signed192> firstTranslation = stackalloc Signed192[3];
        GetRelativeLocalPointNumerators(
            secondCenter,
            firstCenter,
            firstBasis,
            out firstTranslation[0],
            out firstTranslation[1],
            out firstTranslation[2]);
        Span<Signed192> secondTranslation = stackalloc Signed192[3];
        GetRelativeLocalPointNumerators(
            secondCenter,
            firstCenter,
            secondBasis,
            out secondTranslation[0],
            out secondTranslation[1],
            out secondTranslation[2]);

        var best = default(BoxPenetration);
        for (int index = 0; index < 3; index++)
        {
            if (!TryKeepFirstFaceAxis(
                    index,
                    firstHalfExtents,
                    secondHalfExtents,
                    relative,
                    firstTranslation,
                    firstBasis.Denominator,
                    secondBasis.Denominator,
                    relativeBasis.Denominator,
                    ref best)
                || !TryKeepSecondFaceAxis(
                    index,
                    firstHalfExtents,
                    secondHalfExtents,
                    relative,
                    secondTranslation,
                    firstBasis.Denominator,
                    secondBasis.Denominator,
                    relativeBasis.Denominator,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        for (int firstIndex = 0; firstIndex < 3; firstIndex++)
        {
            for (int secondIndex = 0; secondIndex < 3; secondIndex++)
            {
                if (!TryKeepCrossAxis(
                        firstIndex,
                        secondIndex,
                        firstHalfExtents,
                        secondHalfExtents,
                        relative,
                        firstTranslation,
                        firstBasis.Denominator,
                        secondBasis.Denominator,
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
        WideAxis3 axis = best.SecondAxisIndex < 0
            ? firstBasis.GetAxis(best.FirstAxisIndex)
            : best.FirstAxisIndex < 0
                ? secondBasis.GetAxis(best.SecondAxisIndex)
                : WideAxis3.Cross(
                    firstBasis.GetAxis(best.FirstAxisIndex),
                    secondBasis.GetAxis(best.SecondAxisIndex));
        WideAxis3 orientedAxis = best.Negate ? -axis : axis;
        Vector3d normal = WideNormalization.GetNormalized(
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

    private static bool TryKeepFirstFaceAxis(
        int axisIndex,
        Vector3d firstHalfExtents,
        Vector3d secondHalfExtents,
        ReadOnlySpan<Signed192> relative,
        ReadOnlySpan<Signed192> firstTranslation,
        Signed192 firstDenominator,
        Signed192 secondDenominator,
        Signed192 relativeDenominator,
        ref BoxPenetration best)
    {
        Signed320 radius = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(firstHalfExtents[axisIndex]),
                relativeDenominator),
            GetExtentNumerator(
                relative[axisIndex],
                relative[3 + axisIndex],
                relative[6 + axisIndex],
                secondHalfExtents));

        Signed320 centerProjection = WideArithmetic.MultiplySigned192(
            firstTranslation[axisIndex],
            secondDenominator);
        Signed320 overlap = WideArithmetic.SubtractSigned320(
            radius,
            GetMagnitude(centerProjection));
        return TryKeepBoxPenetration(
            axisIndex,
            -1,
            centerProjection.Sign < 0,
            WideArithmetic.MultiplySigned320(overlap, firstDenominator),
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    firstDenominator,
                    firstDenominator)),
            ref best);
    }

    private static bool TryKeepSecondFaceAxis(
        int axisIndex,
        Vector3d firstHalfExtents,
        Vector3d secondHalfExtents,
        ReadOnlySpan<Signed192> relative,
        ReadOnlySpan<Signed192> secondTranslation,
        Signed192 firstDenominator,
        Signed192 secondDenominator,
        Signed192 relativeDenominator,
        ref BoxPenetration best)
    {
        int relativeOffset = axisIndex * 3;
        Signed320 radius = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(secondHalfExtents[axisIndex]),
                relativeDenominator),
            GetExtentNumerator(
                relative[relativeOffset],
                relative[relativeOffset + 1],
                relative[relativeOffset + 2],
                firstHalfExtents));

        Signed320 centerProjection = WideArithmetic.MultiplySigned192(
            secondTranslation[axisIndex],
            firstDenominator);
        Signed320 overlap = WideArithmetic.SubtractSigned320(
            radius,
            GetMagnitude(centerProjection));
        return TryKeepBoxPenetration(
            -1,
            axisIndex,
            centerProjection.Sign < 0,
            WideArithmetic.MultiplySigned320(overlap, secondDenominator),
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    secondDenominator,
                    secondDenominator)),
            ref best);
    }

    private static bool TryKeepCrossAxis(
        int firstAxisIndex,
        int secondAxisIndex,
        Vector3d firstHalfExtents,
        Vector3d secondHalfExtents,
        ReadOnlySpan<Signed192> relative,
        ReadOnlySpan<Signed192> firstTranslation,
        Signed192 firstDenominator,
        Signed192 secondDenominator,
        ref BoxPenetration best)
    {
        int firstNext = (firstAxisIndex + 1) % 3;
        int firstLast = (firstAxisIndex + 2) % 3;
        int secondNext = (secondAxisIndex + 1) % 3;
        int secondLast = (secondAxisIndex + 2) % 3;
        int secondOffset = secondAxisIndex * 3;
        Signed192 firstComponent = relative[secondOffset + firstNext];
        Signed192 secondComponent = relative[secondOffset + firstLast];
        Signed320 squaredAxisLength = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                firstComponent,
                firstComponent),
            WideArithmetic.MultiplySigned192(
                secondComponent,
                secondComponent));
        if (squaredAxisLength.IsZero)
            return true;

        Signed320 radius = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(firstHalfExtents[firstNext]),
                GetMagnitude(secondComponent)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(firstHalfExtents[firstLast]),
                GetMagnitude(firstComponent)));
        radius = WideArithmetic.AddSigned320(
            radius,
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(secondHalfExtents[secondNext]),
                    GetMagnitude(relative[(secondLast * 3) + firstAxisIndex])),
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(secondHalfExtents[secondLast]),
                    GetMagnitude(relative[(secondNext * 3) + firstAxisIndex]))));
        Signed320 centerProjection = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                firstTranslation[firstLast],
                firstComponent),
            WideArithmetic.MultiplySigned192(
                firstTranslation[firstNext],
                secondComponent));
        Signed576 overlap = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(radius, firstDenominator),
            Signed576.ExtendValue(GetMagnitude(centerProjection)));
        return TryKeepBoxPenetration(
            firstAxisIndex,
            secondAxisIndex,
            centerProjection.Sign < 0,
            WideArithmetic.MultiplySigned576(overlap, secondDenominator),
            Signed576.ExtendValue(squaredAxisLength),
            ref best);
    }

    private static bool TryKeepBoxPenetration(
        int firstAxisIndex,
        int secondAxisIndex,
        bool negate,
        Signed576 overlap,
        Signed576 squaredAxisLength,
        ref BoxPenetration best)
    {
        if (overlap.Sign < 0)
            return false;

        if (!best.HasValue
            || CompareBoxDepth(
                overlap,
                squaredAxisLength,
                best.Overlap,
                best.SquaredAxisLength) < 0)
        {
            best = new BoxPenetration(
                firstAxisIndex,
                secondAxisIndex,
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
        WideArithmetic.MultiplyMagnitudes(candidate, candidate, candidateSquared);
        WideArithmetic.MultiplyMagnitudes(current, current, currentSquared);
        WideArithmetic.MultiplyMagnitudes(
            candidateSquared,
            currentAxis,
            candidateScaled);
        WideArithmetic.MultiplyMagnitudes(
            currentSquared,
            candidateAxis,
            currentScaled);
        return WideArithmetic.CompareMagnitudeEqualLength(candidateScaled, currentScaled);
    }
}
