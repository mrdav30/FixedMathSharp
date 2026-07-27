//=======================================================================
// WideFiniteAxisIntersection.CenteredAxes.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <content>
/// Contains logic for computing closest-point intersections between finite axes
/// defined relative to centered (origin-relative) coordinate frames, using
/// wide-precision (Signed320/576/832) arithmetic for numerical stability.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    #region NestedTypes

    private readonly struct CenteredAxesCandidate
    {
        internal readonly Signed576 FirstX;
        internal readonly Signed576 FirstY;
        internal readonly Signed576 FirstZ;
        internal readonly Signed576 SecondX;
        internal readonly Signed576 SecondY;
        internal readonly Signed576 SecondZ;
        internal readonly Signed320 FirstParameterNumerator;
        internal readonly Signed320 SecondParameterNumerator;
        internal readonly Signed320 ParameterDenominator;
        internal readonly Signed832 SquaredDistanceNumerator;

        internal CenteredAxesCandidate(
            Signed576 firstX,
            Signed576 firstY,
            Signed576 firstZ,
            Signed576 secondX,
            Signed576 secondY,
            Signed576 secondZ,
            Signed320 firstParameterNumerator,
            Signed320 secondParameterNumerator,
            Signed320 parameterDenominator,
            Signed832 squaredDistanceNumerator)
        {
            FirstX = firstX;
            FirstY = firstY;
            FirstZ = firstZ;
            SecondX = secondX;
            SecondY = secondY;
            SecondZ = secondZ;
            FirstParameterNumerator = firstParameterNumerator;
            SecondParameterNumerator = secondParameterNumerator;
            ParameterDenominator = parameterDenominator;
            SquaredDistanceNumerator = squaredDistanceNumerator;
        }
    }

    private readonly struct CenteredAxesCandidate2d
    {
        internal readonly Signed576 FirstX;
        internal readonly Signed576 FirstY;
        internal readonly Signed576 SecondX;
        internal readonly Signed576 SecondY;
        internal readonly Signed320 ParameterDenominator;
        internal readonly Signed832 SquaredDistanceNumerator;

        internal CenteredAxesCandidate2d(
            Signed576 firstX,
            Signed576 firstY,
            Signed576 secondX,
            Signed576 secondY,
            Signed320 parameterDenominator,
            Signed832 squaredDistanceNumerator)
        {
            FirstX = firstX;
            FirstY = firstY;
            SecondX = secondX;
            SecondY = secondY;
            ParameterDenominator = parameterDenominator;
            SquaredDistanceNumerator = squaredDistanceNumerator;
        }
    }

    #endregion

    internal static bool TryGetDistanceToCenteredAxis(
        Vector2d point,
        Vector2d center,
        Vector2d axis,
        Fixed64 axisLength,
        out Fixed64 distance) =>
        TryGetDistanceToCenteredCapsule(
            point,
            center,
            axis,
            axisLength,
            Fixed64.Zero,
            out distance);

    internal static bool TryGetDistanceToCenteredAxis(
        Vector3d point,
        Vector3d center,
        Vector3d axis,
        Fixed64 axisLength,
        out Fixed64 distance) =>
        TryGetDistanceToCenteredCapsule(
            point,
            center,
            axis,
            axisLength,
            Fixed64.Zero,
            out distance);

    internal static bool TryGetDistanceBetweenCenteredAxes(
        Vector2d firstCenter,
        Vector2d firstAxis,
        Fixed64 firstLength,
        Vector2d secondCenter,
        Vector2d secondAxis,
        Fixed64 secondLength,
        out Fixed64 distance)
    {
        GetCenteredAxesSquaredDistance(
            firstCenter,
            firstAxis,
            firstLength,
            secondCenter,
            secondAxis,
            secondLength,
            out Signed832 squaredNumerator,
            out Signed320 denominator);
        return TryRoundCenteredAxesDistance(
            squaredNumerator,
            denominator,
            out distance);
    }

    internal static bool TryGetDistanceBetweenCenteredAxes(
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        out Fixed64 distance)
    {
        GetCenteredAxesSquaredDistance(
            firstCenter,
            firstAxis,
            firstLength,
            secondCenter,
            secondAxis,
            secondLength,
            out Signed832 squaredNumerator,
            out Signed320 denominator);
        return TryRoundCenteredAxesDistance(
            squaredNumerator,
            denominator,
            out distance);
    }

    internal static bool DoCenteredCapsulesOverlap(
        Vector2d firstCenter,
        Vector2d firstAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector2d secondCenter,
        Vector2d secondAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius)
    {
        GetCenteredAxesSquaredDistance(
            firstCenter,
            firstAxis,
            firstLength,
            secondCenter,
            secondAxis,
            secondLength,
            out Signed832 squaredNumerator,
            out Signed320 denominator);
        return IsWithinCombinedRadius(
            squaredNumerator,
            denominator,
            firstRadius,
            secondRadius);
    }

    internal static bool DoCenteredCapsulesOverlap(
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius)
    {
        GetCenteredAxesSquaredDistance(
            firstCenter,
            firstAxis,
            firstLength,
            secondCenter,
            secondAxis,
            secondLength,
            out Signed832 squaredNumerator,
            out Signed320 denominator);
        return IsWithinCombinedRadius(
            squaredNumerator,
            denominator,
            firstRadius,
            secondRadius);
    }

    internal static bool TryGetCenteredCapsulesContact(
        Vector2d firstCenter,
        Fixed64 firstAnchorRotation,
        Vector2d firstAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector2d secondCenter,
        Fixed64 secondAnchorRotation,
        Vector2d secondAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        Vector2d fallbackNormal,
        out FixedContactAnchors2d contact)
    {
        GetCenteredAxesCandidate(
            firstCenter,
            firstAxis,
            firstLength,
            secondCenter,
            secondAxis,
            secondLength,
            out CenteredAxesCandidate2d candidate);
        if (!IsWithinCombinedRadius(
                candidate.SquaredDistanceNumerator,
                candidate.ParameterDenominator,
                firstRadius,
                secondRadius))
        {
            contact = default;
            return false;
        }

        Vector2d normal = WideGeometry.GetNormalized(
            WideArithmetic.SubtractSigned576(candidate.SecondX, candidate.FirstX),
            WideArithmetic.SubtractSigned576(candidate.SecondY, candidate.FirstY));
        if (normal == Vector2d.Zero)
            normal = fallbackNormal;

        GetLocalRadialFeature(
            normal,
            firstRadius,
            firstAnchorRotation,
            out Vector2d firstLocalRadialDirection,
            out Vector2d firstRadialOffset);
        GetLocalRadialFeature(
            -normal,
            secondRadius,
            secondAnchorRotation,
            out Vector2d secondLocalRadialDirection,
            out Vector2d secondRadialOffset);
        Vector2d firstAxisOffset = GetCenteredAxisLocalOffset(
            candidate.FirstX,
            candidate.FirstY,
            firstCenter,
            candidate.ParameterDenominator,
            firstAnchorRotation);
        Vector2d secondAxisOffset = GetCenteredAxisLocalOffset(
            candidate.SecondX,
            candidate.SecondY,
            secondCenter,
            candidate.ParameterDenominator,
            secondAnchorRotation);
        _ = TryGetCenteredCapsulesDepth(
            candidate.SquaredDistanceNumerator,
            candidate.ParameterDenominator,
            firstRadius,
            secondRadius,
            out Fixed64 depth,
            out bool depthIsClamped);
        contact = new FixedContactAnchors2d(
            new FixedPointAnchor2d(
                firstCenter,
                firstAnchorRotation,
                firstAxisOffset,
                firstRadialOffset,
                FixedPointAnchorTerm2d.CreateRadialSupport(
                    firstLocalRadialDirection,
                    firstRadius,
                    firstRadialOffset)),
            new FixedPointAnchor2d(
                secondCenter,
                secondAnchorRotation,
                secondAxisOffset,
                secondRadialOffset,
                FixedPointAnchorTerm2d.CreateRadialSupport(
                    secondLocalRadialDirection,
                    secondRadius,
                    secondRadialOffset)),
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    internal static bool TryGetCenteredCapsulesContact(
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Vector3d firstLocalAxis,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Vector3d secondLocalAxis,
        Vector3d secondAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        Vector3d fallbackNormal,
        out FixedContactAnchors contact)
    {
        GetCenteredAxesCandidate(
            firstCenter,
            firstAxis,
            firstLength,
            secondCenter,
            secondAxis,
            secondLength,
            out CenteredAxesCandidate candidate);
        Signed576 differenceX = WideArithmetic.SubtractSigned576(
            candidate.SecondX,
            candidate.FirstX);
        Signed576 differenceY = WideArithmetic.SubtractSigned576(
            candidate.SecondY,
            candidate.FirstY);
        Signed576 differenceZ = WideArithmetic.SubtractSigned576(
            candidate.SecondZ,
            candidate.FirstZ);
        Vector3d normal = WideGeometry.GetNormalized(
            differenceX,
            differenceY,
            differenceZ);
        if (normal.IsZero)
            normal = fallbackNormal;

        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero,
            firstLocalAxis.X,
            candidate.FirstParameterNumerator,
            candidate.ParameterDenominator,
            out Fixed64 firstX);
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero,
            firstLocalAxis.Y,
            candidate.FirstParameterNumerator,
            candidate.ParameterDenominator,
            out Fixed64 firstY);
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero,
            firstLocalAxis.Z,
            candidate.FirstParameterNumerator,
            candidate.ParameterDenominator,
            out Fixed64 firstZ);
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero,
            secondLocalAxis.X,
            candidate.SecondParameterNumerator,
            candidate.ParameterDenominator,
            out Fixed64 secondX);
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero,
            secondLocalAxis.Y,
            candidate.SecondParameterNumerator,
            candidate.ParameterDenominator,
            out Fixed64 secondY);
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero,
            secondLocalAxis.Z,
            candidate.SecondParameterNumerator,
            candidate.ParameterDenominator,
            out Fixed64 secondZ);
        _ = TryGetCenteredCapsulesDepth(
            candidate.SquaredDistanceNumerator,
            candidate.ParameterDenominator,
            firstRadius,
            secondRadius,
            out Fixed64 depth,
            out bool depthIsClamped);
        depth = FixedMath.Max(depth, Fixed64.Zero);
        Vector3d firstLocalRadialDirection = WideGeometry.GetNormalized(
            firstRotation.Inverse().Rotate(normal));
        Vector3d secondLocalRadialDirection = WideGeometry.GetNormalized(
            secondRotation.Inverse().Rotate(-normal));
        Vector3d firstRadialOffset =
            firstLocalRadialDirection * firstRadius;
        Vector3d secondRadialOffset =
            secondLocalRadialDirection * secondRadius;
        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                firstCenter,
                firstRotation,
                new Vector3d(firstX, firstY, firstZ),
                firstRadialOffset,
                FixedPointAnchorTerm3d.CreateRadialSupport(
                    firstLocalRadialDirection,
                    firstRadius,
                    firstRadialOffset)),
            new FixedPointAnchor(
                secondCenter,
                secondRotation,
                new Vector3d(secondX, secondY, secondZ),
                secondRadialOffset,
                FixedPointAnchorTerm3d.CreateRadialSupport(
                    secondLocalRadialDirection,
                    secondRadius,
                    secondRadialOffset)),
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    internal static bool TryGetClosestPointsBetweenCenteredAxes(
        Vector2d firstCenter,
        Vector2d firstAxis,
        Fixed64 firstLength,
        Vector2d secondCenter,
        Vector2d secondAxis,
        Fixed64 secondLength,
        out Vector2d firstPoint,
        out Vector2d secondPoint)
    {
        GetClosestCenteredAxisParameters(
            GetDot(firstAxis, Vector2d.Zero, firstAxis, Vector2d.Zero),
            GetDot(firstAxis, Vector2d.Zero, secondAxis, Vector2d.Zero),
            GetDot(secondAxis, Vector2d.Zero, secondAxis, Vector2d.Zero),
            GetDot(firstCenter, secondCenter, firstAxis, Vector2d.Zero),
            GetDot(firstCenter, secondCenter, secondAxis, Vector2d.Zero),
            firstLength,
            secondLength,
            out Signed320 firstNumerator,
            out Signed320 secondNumerator,
            out Signed320 denominator);

        bool representable = TryGetCenteredAxisPointCoordinate(
            firstCenter.X, firstAxis.X, firstNumerator, denominator, out Fixed64 firstX);
        representable &= TryGetCenteredAxisPointCoordinate(
            firstCenter.Y, firstAxis.Y, firstNumerator, denominator, out Fixed64 firstY);
        representable &= TryGetCenteredAxisPointCoordinate(
            secondCenter.X, secondAxis.X, secondNumerator, denominator, out Fixed64 secondX);
        representable &= TryGetCenteredAxisPointCoordinate(
            secondCenter.Y, secondAxis.Y, secondNumerator, denominator, out Fixed64 secondY);
        if (!representable)
        {
            firstPoint = default;
            secondPoint = default;
            return false;
        }

        firstPoint = new Vector2d(firstX, firstY);
        secondPoint = new Vector2d(secondX, secondY);
        return true;
    }

    internal static bool TryGetClosestPointsBetweenCenteredAxes(
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        out Vector3d firstPoint,
        out Vector3d secondPoint)
    {
        GetClosestCenteredAxisParameters(
            GetDot(firstAxis, Vector3d.Zero, firstAxis, Vector3d.Zero),
            GetDot(firstAxis, Vector3d.Zero, secondAxis, Vector3d.Zero),
            GetDot(secondAxis, Vector3d.Zero, secondAxis, Vector3d.Zero),
            GetDot(firstCenter, secondCenter, firstAxis, Vector3d.Zero),
            GetDot(firstCenter, secondCenter, secondAxis, Vector3d.Zero),
            firstLength,
            secondLength,
            out Signed320 firstNumerator,
            out Signed320 secondNumerator,
            out Signed320 denominator);

        bool representable = TryGetCenteredAxisPointCoordinate(
            firstCenter.X, firstAxis.X, firstNumerator, denominator, out Fixed64 firstX);
        representable &= TryGetCenteredAxisPointCoordinate(
            firstCenter.Y, firstAxis.Y, firstNumerator, denominator, out Fixed64 firstY);
        representable &= TryGetCenteredAxisPointCoordinate(
            firstCenter.Z, firstAxis.Z, firstNumerator, denominator, out Fixed64 firstZ);
        representable &= TryGetCenteredAxisPointCoordinate(
            secondCenter.X, secondAxis.X, secondNumerator, denominator, out Fixed64 secondX);
        representable &= TryGetCenteredAxisPointCoordinate(
            secondCenter.Y, secondAxis.Y, secondNumerator, denominator, out Fixed64 secondY);
        representable &= TryGetCenteredAxisPointCoordinate(
            secondCenter.Z, secondAxis.Z, secondNumerator, denominator, out Fixed64 secondZ);
        if (!representable)
        {
            firstPoint = default;
            secondPoint = default;
            return false;
        }

        firstPoint = new Vector3d(firstX, firstY, firstZ);
        secondPoint = new Vector3d(secondX, secondY, secondZ);
        return true;
    }

    internal static bool TryGetClosestOffsetsBetweenCenteredAxes(
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        out Vector3d firstCenterOffset,
        out Vector3d secondCenterOffset)
    {
        GetClosestCenteredAxisParameters(
            GetDot(firstAxis, Vector3d.Zero, firstAxis, Vector3d.Zero),
            GetDot(firstAxis, Vector3d.Zero, secondAxis, Vector3d.Zero),
            GetDot(secondAxis, Vector3d.Zero, secondAxis, Vector3d.Zero),
            GetDot(firstCenter, secondCenter, firstAxis, Vector3d.Zero),
            GetDot(firstCenter, secondCenter, secondAxis, Vector3d.Zero),
            firstLength,
            secondLength,
            out Signed320 firstNumerator,
            out Signed320 secondNumerator,
            out Signed320 denominator);

        // A centered-axis offset cannot exceed half of its representable full
        // length, so valid normalized axes always narrow successfully here.
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero, firstAxis.X, firstNumerator, denominator, out Fixed64 firstX);
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero, firstAxis.Y, firstNumerator, denominator, out Fixed64 firstY);
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero, firstAxis.Z, firstNumerator, denominator, out Fixed64 firstZ);
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero, secondAxis.X, secondNumerator, denominator, out Fixed64 secondX);
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero, secondAxis.Y, secondNumerator, denominator, out Fixed64 secondY);
        _ = TryGetCenteredAxisPointCoordinate(
            Fixed64.Zero, secondAxis.Z, secondNumerator, denominator, out Fixed64 secondZ);

        firstCenterOffset = new Vector3d(firstX, firstY, firstZ);
        secondCenterOffset = new Vector3d(secondX, secondY, secondZ);
        return true;
    }

    internal static Vector3d GetClosestDirectionBetweenCenteredAxes(
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength)
    {
        GetCenteredAxesCandidate(
            firstCenter,
            firstAxis,
            firstLength,
            secondCenter,
            secondAxis,
            secondLength,
            out CenteredAxesCandidate candidate);
        return WideGeometry.GetNormalized(
            WideArithmetic.SubtractSigned576(candidate.SecondX, candidate.FirstX),
            WideArithmetic.SubtractSigned576(candidate.SecondY, candidate.FirstY),
            WideArithmetic.SubtractSigned576(candidate.SecondZ, candidate.FirstZ));
    }
}
