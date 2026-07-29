//=======================================================================
// WideFiniteAxisIntersection.CenteredCapsule.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides high-precision surface point and offset calculations for capsules
/// centered at the origin (or a given center), using a fixed axis and radius.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static bool TryGetSurfacePointOnCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d normal,
        out Vector2d surfacePoint)
    {
        GetClosestCenteredAxisRatio(
            point,
            center,
            axisDirection,
            axisLength,
            out Signed192 numerator,
            out Signed192 denominator);
        if (!TryGetCenteredCapsuleSurfaceCoordinate(
                center.X, axisDirection.X, numerator, denominator, normal.X, radius, out Fixed64 x)
            || !TryGetCenteredCapsuleSurfaceCoordinate(
                center.Y, axisDirection.Y, numerator, denominator, normal.Y, radius, out Fixed64 y))
        {
            surfacePoint = default;
            return false;
        }

        surfacePoint = new Vector2d(x, y);
        return true;
    }

    internal static bool TryGetSurfaceOffsetOnCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d normal,
        out Vector2d surfaceOffset)
    {
        GetClosestCenteredAxisRatio(
            point,
            center,
            axisDirection,
            axisLength,
            out Signed192 numerator,
            out Signed192 denominator);
        bool representable =
            TryGetCenteredCapsuleSurfaceCoordinate(
                Fixed64.Zero,
                axisDirection.X,
                numerator,
                denominator,
                normal.X,
                radius,
                out Fixed64 x)
            & TryGetCenteredCapsuleSurfaceCoordinate(
                Fixed64.Zero,
                axisDirection.Y,
                numerator,
                denominator,
                normal.Y,
                radius,
                out Fixed64 y);
        surfaceOffset = representable ? new Vector2d(x, y) : default;
        return representable;
    }

    internal static bool TryGetSurfaceOffsetOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d normal,
        out Vector3d surfaceOffset)
    {
        GetClosestCenteredAxisRatio(
            point,
            center,
            axisDirection,
            axisLength,
            out Signed192 numerator,
            out Signed192 denominator);
        bool representable =
            TryGetCenteredCapsuleSurfaceCoordinate(
                Fixed64.Zero, axisDirection.X, numerator, denominator, normal.X, radius, out Fixed64 x)
            & TryGetCenteredCapsuleSurfaceCoordinate(
                Fixed64.Zero, axisDirection.Y, numerator, denominator, normal.Y, radius, out Fixed64 y)
            & TryGetCenteredCapsuleSurfaceCoordinate(
                Fixed64.Zero, axisDirection.Z, numerator, denominator, normal.Z, radius, out Fixed64 z);
        surfaceOffset = representable ? new Vector3d(x, y, z) : default;
        return representable;
    }

    internal static bool TryGetSurfacePointOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d normal,
        out Vector3d surfacePoint)
    {
        GetClosestCenteredAxisRatio(
            point,
            center,
            axisDirection,
            axisLength,
            out Signed192 numerator,
            out Signed192 denominator);
        if (!TryGetCenteredCapsuleSurfaceCoordinate(
                center.X, axisDirection.X, numerator, denominator, normal.X, radius, out Fixed64 x)
            || !TryGetCenteredCapsuleSurfaceCoordinate(
                center.Y, axisDirection.Y, numerator, denominator, normal.Y, radius, out Fixed64 y)
            || !TryGetCenteredCapsuleSurfaceCoordinate(
                center.Z, axisDirection.Z, numerator, denominator, normal.Z, radius, out Fixed64 z))
        {
            surfacePoint = default;
            return false;
        }

        surfacePoint = new Vector3d(x, y, z);
        return true;
    }

    internal static bool ContainsPointInCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        bool strict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector2d.Zero, axisDirection, Vector2d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector2d.Zero);
        return IsCenteredCapsulePointContained(
            point,
            center,
            axisDirection,
            axisLength,
            GetDot(point, center, point, center),
            axisProjection,
            axisLengthSquared,
            GetSquaredRadius(expandedRadius),
            expandedRadius,
            GetCenteredAxialExtent(axisLengthSquared, axisLength, Fixed64.Zero),
            strict);
    }

    internal static bool ContainsPointInCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        bool strict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector3d.Zero);
        return IsCenteredCapsulePointContained(
            point,
            center,
            axisDirection,
            axisLength,
            GetDot(point, center, point, center),
            axisProjection,
            axisLengthSquared,
            GetSquaredRadius(expandedRadius),
            expandedRadius,
            GetCenteredAxialExtent(axisLengthSquared, axisLength, Fixed64.Zero),
            strict);
    }

    internal static Vector2d GetDirectionFromCenteredAxis(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength)
    {
        Signed192 differenceX = GetComponentDifference(point.X, center.X);
        Signed192 differenceY = GetComponentDifference(point.Y, center.Y);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector2d.Zero, axisDirection, Vector2d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector2d.Zero);
        int cap = GetCenteredCap(
            axisProjection,
            GetCenteredAxialExtent(axisLengthSquared, axisLength, Fixed64.Zero));
        if (cap != 0)
        {
            return WideNormalization.GetNormalized(
                GetCenteredCapOffset(point.X, center.X, axisDirection.X, axisLength, cap > 0),
                GetCenteredCapOffset(point.Y, center.Y, axisDirection.Y, axisLength, cap > 0));
        }

        Signed192 axisX = Signed192.Signed(axisDirection.X.m_rawValue);
        Signed192 axisY = Signed192.Signed(axisDirection.Y.m_rawValue);
        return WideNormalization.GetNormalized(
            GetRadialComponent(axisLengthSquared, differenceX, axisX, axisProjection),
            GetRadialComponent(axisLengthSquared, differenceY, axisY, axisProjection));
    }

    internal static Vector3d GetDirectionFromCenteredAxis(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength)
    {
        Signed192 differenceX = GetComponentDifference(point.X, center.X);
        Signed192 differenceY = GetComponentDifference(point.Y, center.Y);
        Signed192 differenceZ = GetComponentDifference(point.Z, center.Z);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector3d.Zero);
        int cap = GetCenteredCap(
            axisProjection,
            GetCenteredAxialExtent(axisLengthSquared, axisLength, Fixed64.Zero));
        if (cap != 0)
        {
            return WideNormalization.GetNormalized(
                GetCenteredCapOffset(point.X, center.X, axisDirection.X, axisLength, cap > 0),
                GetCenteredCapOffset(point.Y, center.Y, axisDirection.Y, axisLength, cap > 0),
                GetCenteredCapOffset(point.Z, center.Z, axisDirection.Z, axisLength, cap > 0));
        }

        Signed192 axisX = Signed192.Signed(axisDirection.X.m_rawValue);
        Signed192 axisY = Signed192.Signed(axisDirection.Y.m_rawValue);
        Signed192 axisZ = Signed192.Signed(axisDirection.Z.m_rawValue);
        return WideNormalization.GetNormalized(
            GetRadialComponent(axisLengthSquared, differenceX, axisX, axisProjection),
            GetRadialComponent(axisLengthSquared, differenceY, axisY, axisProjection),
            GetRadialComponent(axisLengthSquared, differenceZ, axisZ, axisProjection));
    }

    internal static bool TryGetCapsuleInterval(
        FixedSegment2d query,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector2d.Zero, axisDirection, Vector2d.Zero);
        Signed192 queryLengthSquared = GetDot(query.End, query.Start, query.End, query.Start);
        Signed192 startDistanceSquared = GetDot(query.Start, center, query.Start, center);
        Signed192 directionsDot = GetDot(query.End, query.Start, axisDirection, Vector2d.Zero);
        Signed192 startAxisProjection = GetDot(query.Start, center, axisDirection, Vector2d.Zero);
        Signed192 endAxisProjection = GetDot(query.End, center, axisDirection, Vector2d.Zero);
        Signed192 startDirectionProjection = GetDot(query.Start, center, query.End, query.Start);
        Signed320 axialExtent = GetCenteredAxialExtent(
            axisLengthSquared,
            axisLength,
            Fixed64.Zero);

        startContained = IsCenteredCapsulePointContained(
            query.Start,
            center,
            axisDirection,
            axisLength,
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared,
            squaredRadius,
            expandedRadius,
            axialExtent,
            strict: false);
        endContainedStrict = IsCenteredCapsulePointContained(
            query.End,
            center,
            axisDirection,
            axisLength,
            GetDot(query.End, center, query.End, center),
            endAxisProjection,
            axisLengthSquared,
            squaredRadius,
            expandedRadius,
            axialExtent,
            strict: true);

        bool found = false;
        entry = default;
        exit = default;
        if (TryGetCenteredAxialInterval(
                startAxisProjection,
                directionsDot,
                axialExtent,
                Fixed64.One,
                out RationalBound320 lower,
                out RationalBound320 upper))
        {
            found = TryGetFiniteAxisInterval(
                queryLengthSquared,
                axisLengthSquared,
                startDistanceSquared,
                directionsDot,
                startAxisProjection,
                startDirectionProjection,
                expandedRadius,
                lower,
                upper,
                Fixed64.One,
                out entry,
                out exit);
        }

        MergeCenteredCapInterval(
            query,
            center,
            axisDirection,
            axisLength,
            expandedRadius,
            positiveCap: false,
            ref found,
            ref entry,
            ref exit);
        MergeCenteredCapInterval(
            query,
            center,
            axisDirection,
            axisLength,
            expandedRadius,
            positiveCap: true,
            ref found,
            ref entry,
            ref exit);
        return found;
    }

    internal static bool TryGetCapsuleInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 queryLengthSquared = GetDot(query.End, query.Start, query.End, query.Start);
        Signed192 startDistanceSquared = GetDot(query.Start, center, query.Start, center);
        Signed192 directionsDot = GetDot(query.End, query.Start, axisDirection, Vector3d.Zero);
        Signed192 startAxisProjection = GetDot(query.Start, center, axisDirection, Vector3d.Zero);
        Signed192 endAxisProjection = GetDot(query.End, center, axisDirection, Vector3d.Zero);
        Signed192 startDirectionProjection = GetDot(query.Start, center, query.End, query.Start);
        Signed320 axialExtent = GetCenteredAxialExtent(
            axisLengthSquared,
            axisLength,
            Fixed64.Zero);

        startContained = IsCenteredCapsulePointContained(
            query.Start,
            center,
            axisDirection,
            axisLength,
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared,
            squaredRadius,
            expandedRadius,
            axialExtent,
            strict: false);
        endContainedStrict = IsCenteredCapsulePointContained(
            query.End,
            center,
            axisDirection,
            axisLength,
            GetDot(query.End, center, query.End, center),
            endAxisProjection,
            axisLengthSquared,
            squaredRadius,
            expandedRadius,
            axialExtent,
            strict: true);

        bool found = false;
        entry = default;
        exit = default;
        if (TryGetCenteredAxialInterval(
                startAxisProjection,
                directionsDot,
                axialExtent,
                Fixed64.One,
                out RationalBound320 lower,
                out RationalBound320 upper))
        {
            found = TryGetFiniteAxisInterval(
                queryLengthSquared,
                axisLengthSquared,
                startDistanceSquared,
                directionsDot,
                startAxisProjection,
                startDirectionProjection,
                expandedRadius,
                lower,
                upper,
                Fixed64.One,
                out entry,
                out exit);
        }

        MergeCenteredCapInterval(
            query,
            center,
            axisDirection,
            axisLength,
            expandedRadius,
            positiveCap: false,
            ref found,
            ref entry,
            ref exit);
        MergeCenteredCapInterval(
            query,
            center,
            axisDirection,
            axisLength,
            expandedRadius,
            positiveCap: true,
            ref found,
            ref entry,
            ref exit);
        return found;
    }

    private static bool IsCenteredCapsulePointContained(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Signed192 distanceSquared,
        Signed192 axisProjection,
        Signed192 axisLengthSquared,
        Signed192 squaredRadius,
        Signed192 expandedRadius,
        Signed320 axialExtent,
        bool strict)
    {
        int cap = GetCenteredCap(axisProjection, axialExtent);
        if (cap == 0)
        {
            int radialSign = GetRadialConstant(
                distanceSquared,
                axisProjection,
                axisLengthSquared,
                squaredRadius).Sign;
            return strict ? radialSign < 0 : radialSign <= 0;
        }

        int capSign = GetCenteredCapConstant(
            point,
            center,
            axisDirection,
            axisLength,
            expandedRadius,
            positiveCap: cap > 0).Sign;
        return strict ? capSign < 0 : capSign <= 0;
    }

    private static bool IsCenteredCapsulePointContained(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Signed192 distanceSquared,
        Signed192 axisProjection,
        Signed192 axisLengthSquared,
        Signed192 squaredRadius,
        Signed192 expandedRadius,
        Signed320 axialExtent,
        bool strict)
    {
        int cap = GetCenteredCap(axisProjection, axialExtent);
        if (cap == 0)
        {
            int radialSign = GetRadialConstant(
                distanceSquared,
                axisProjection,
                axisLengthSquared,
                squaredRadius).Sign;
            return strict ? radialSign < 0 : radialSign <= 0;
        }

        int capSign = GetCenteredCapConstant(
            point,
            center,
            axisDirection,
            axisLength,
            expandedRadius,
            positiveCap: cap > 0).Sign;
        return strict ? capSign < 0 : capSign <= 0;
    }

    private static int GetCenteredCap(Signed192 axisProjection, Signed320 axialExtent)
    {
        Signed320 scaledProjection = WideArithmetic.MultiplySigned192(DoubleParameterScale, axisProjection);
        if (WideArithmetic.AddSigned320(scaledProjection, axialExtent).Sign <= 0)
            return -1;
        return WideArithmetic.SubtractSigned320(scaledProjection, axialExtent).Sign >= 0
            ? 1
            : 0;
    }

    private static void MergeCenteredCapInterval(
        FixedSegment2d query,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Signed192 expandedRadius,
        bool positiveCap,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        GetCenteredCapPolynomial(
            query,
            center,
            axisDirection,
            axisLength,
            expandedRadius,
            positiveCap,
            out Signed320 coefficient,
            out Signed320 projection,
            out Signed320 constant);
        Merge(
            TrySolveUnitQuadratic(coefficient, projection, constant, out Fixed64 capEntry, out Fixed64 capExit),
            capEntry,
            capExit,
            ref found,
            ref entry,
            ref exit);
    }

    private static void MergeCenteredCapInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Signed192 expandedRadius,
        bool positiveCap,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        GetCenteredCapPolynomial(
            query,
            center,
            axisDirection,
            axisLength,
            expandedRadius,
            positiveCap,
            out Signed320 coefficient,
            out Signed320 projection,
            out Signed320 constant);
        Merge(
            TrySolveUnitQuadratic(coefficient, projection, constant, out Fixed64 capEntry, out Fixed64 capExit),
            capEntry,
            capExit,
            ref found,
            ref entry,
            ref exit);
    }

    private static void GetCenteredCapPolynomial(
        FixedSegment2d query,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Signed192 expandedRadius,
        bool positiveCap,
        out Signed320 coefficient,
        out Signed320 projection,
        out Signed320 constant)
    {
        Signed192 x = GetCenteredCapOffset(
            query.Start.X,
            center.X,
            axisDirection.X,
            axisLength,
            positiveCap);
        Signed192 y = GetCenteredCapOffset(
            query.Start.Y,
            center.Y,
            axisDirection.Y,
            axisLength,
            positiveCap);
        Signed192 velocityX = GetScaledDifference(query.End.X, query.Start.X);
        Signed192 velocityY = GetScaledDifference(query.End.Y, query.Start.Y);
        Signed192 scaledRadius = ScaleByCenteredAxis(expandedRadius);

        coefficient = WideArithmetic.AddProducts(velocityX, velocityX, velocityY, velocityY);
        projection = WideArithmetic.AddProducts(x, velocityX, y, velocityY);
        constant = WideArithmetic.SubtractSigned320(
            WideArithmetic.AddProducts(x, x, y, y),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
    }

    private static void GetCenteredCapPolynomial(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Signed192 expandedRadius,
        bool positiveCap,
        out Signed320 coefficient,
        out Signed320 projection,
        out Signed320 constant)
    {
        Signed192 x = GetCenteredCapOffset(
            query.Start.X,
            center.X,
            axisDirection.X,
            axisLength,
            positiveCap);
        Signed192 y = GetCenteredCapOffset(
            query.Start.Y,
            center.Y,
            axisDirection.Y,
            axisLength,
            positiveCap);
        Signed192 z = GetCenteredCapOffset(
            query.Start.Z,
            center.Z,
            axisDirection.Z,
            axisLength,
            positiveCap);
        Signed192 velocityX = GetScaledDifference(query.End.X, query.Start.X);
        Signed192 velocityY = GetScaledDifference(query.End.Y, query.Start.Y);
        Signed192 velocityZ = GetScaledDifference(query.End.Z, query.Start.Z);
        Signed192 scaledRadius = ScaleByCenteredAxis(expandedRadius);

        coefficient = WideArithmetic.AddProducts(velocityX, velocityX, velocityY, velocityY, velocityZ, velocityZ);
        projection = WideArithmetic.AddProducts(x, velocityX, y, velocityY, z, velocityZ);
        constant = WideArithmetic.SubtractSigned320(
            WideArithmetic.AddProducts(x, x, y, y, z, z),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
    }

    private static Signed320 GetCenteredCapConstant(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Signed192 expandedRadius,
        bool positiveCap)
    {
        Signed192 x = GetCenteredCapOffset(
            point.X,
            center.X,
            axisDirection.X,
            axisLength,
            positiveCap);
        Signed192 y = GetCenteredCapOffset(
            point.Y,
            center.Y,
            axisDirection.Y,
            axisLength,
            positiveCap);
        Signed192 scaledRadius = ScaleByCenteredAxis(expandedRadius);
        return WideArithmetic.SubtractSigned320(
            WideArithmetic.AddProducts(x, x, y, y),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
    }

    private static Signed320 GetCenteredCapConstant(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Signed192 expandedRadius,
        bool positiveCap)
    {
        Signed192 x = GetCenteredCapOffset(
            point.X,
            center.X,
            axisDirection.X,
            axisLength,
            positiveCap);
        Signed192 y = GetCenteredCapOffset(
            point.Y,
            center.Y,
            axisDirection.Y,
            axisLength,
            positiveCap);
        Signed192 z = GetCenteredCapOffset(
            point.Z,
            center.Z,
            axisDirection.Z,
            axisLength,
            positiveCap);
        Signed192 scaledRadius = ScaleByCenteredAxis(expandedRadius);
        return WideArithmetic.SubtractSigned320(
            WideArithmetic.AddProducts(x, x, y, y, z, z),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
    }

    private static Signed192 GetCenteredCapOffset(
        Fixed64 point,
        Fixed64 center,
        Fixed64 axisDirection,
        Fixed64 axisLength,
        bool positiveCap)
    {
        Signed192 difference = WideArithmetic.SubtractSigned192(
            Signed192.Signed(point.m_rawValue),
            Signed192.Signed(center.m_rawValue));
        Signed320 scaledDifference = WideArithmetic.MultiplySigned192(difference, DoubleParameterScale);
        Signed320 directionalOffset = WideArithmetic.MultiplySigned192(
            Signed192.Signed(axisDirection.m_rawValue),
            Signed192.Signed(axisLength.m_rawValue));
        return Signed192.NarrowValue(positiveCap
            ? WideArithmetic.SubtractSigned320(scaledDifference, directionalOffset)
            : WideArithmetic.AddSigned320(scaledDifference, directionalOffset));
    }

    private static Signed320 GetRadialComponent(
        Signed192 axisLengthSquared,
        Signed192 difference,
        Signed192 axisComponent,
        Signed192 axisProjection) =>
        WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(axisLengthSquared, difference),
            WideArithmetic.MultiplySigned192(axisComponent, axisProjection));

    private static void GetClosestCenteredAxisRatio(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        out Signed192 numerator,
        out Signed192 denominator)
    {
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector2d.Zero, axisDirection, Vector2d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector2d.Zero);
        int cap = GetCenteredCap(
            axisProjection,
            GetCenteredAxialExtent(axisLengthSquared, axisLength, Fixed64.Zero));
        if (cap == 0)
        {
            numerator = axisProjection;
            denominator = axisLengthSquared;
            return;
        }

        numerator = Signed192.Signed(
            cap < 0 ? -axisLength.m_rawValue : axisLength.m_rawValue);
        denominator = DoubleParameterScale;
    }

    private static void GetClosestCenteredAxisRatio(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        out Signed192 numerator,
        out Signed192 denominator)
    {
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector3d.Zero);
        int cap = GetCenteredCap(
            axisProjection,
            GetCenteredAxialExtent(axisLengthSquared, axisLength, Fixed64.Zero));
        if (cap == 0)
        {
            numerator = axisProjection;
            denominator = axisLengthSquared;
            return;
        }

        numerator = Signed192.Signed(
            cap < 0 ? -axisLength.m_rawValue : axisLength.m_rawValue);
        denominator = DoubleParameterScale;
    }

    private static bool TryGetCenteredCapsuleSurfaceCoordinate(
        Fixed64 center,
        Fixed64 axisDirection,
        Signed192 axisNumerator,
        Signed192 axisDenominator,
        Fixed64 normal,
        Fixed64 radius,
        out Fixed64 coordinate)
    {
        Signed320 centerTerm = MultiplyThreeToSigned320(
            Signed192.Signed(center.m_rawValue),
            axisDenominator,
            ParameterScale);
        Signed320 axisTerm = MultiplyThreeToSigned320(
            Signed192.Signed(axisDirection.m_rawValue),
            axisNumerator,
            ParameterScale);
        Signed320 radialTerm = MultiplyThreeToSigned320(
            Signed192.Signed(normal.m_rawValue),
            Signed192.Signed(radius.m_rawValue),
            axisDenominator);
        Signed320 denominator = MultiplyThreeToSigned320(
            axisDenominator,
            ParameterScale,
            ParameterScale);
        Signed320 numerator = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(centerTerm, axisTerm),
            radialTerm);
        return Fixed64.TryGetSignedRatio(numerator, denominator, out coordinate);
    }

    private static Signed320 MultiplyThreeToSigned320(
        Signed192 first,
        Signed192 second,
        Signed192 third) =>
        // Centered-capsule surface terms use at most 194 signed bits. The upper
        // four words are therefore sign extension before this mechanical narrow.
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(first, second),
            Signed320.ExtendValue(third)));

    private static Signed320 GetCenteredAxisOffsetComponent(
        Fixed64 point,
        Fixed64 center,
        Fixed64 axisDirection,
        Signed192 axisNumerator,
        Signed192 axisDenominator) =>
        WideArithmetic.MultiplySubtract(
            GetComponentDifference(point, center),
            axisDenominator,
            Signed192.Signed(axisDirection.m_rawValue),
            axisNumerator);

    private static Signed192 GetComponentDifference(Fixed64 point, Fixed64 center) =>
        WideArithmetic.SubtractSigned192(
            Signed192.Signed(point.m_rawValue),
            Signed192.Signed(center.m_rawValue));

    private static Signed192 GetScaledDifference(Fixed64 end, Fixed64 start) =>
        ScaleByCenteredAxis(WideArithmetic.SubtractSigned192(
            Signed192.Signed(end.m_rawValue),
            Signed192.Signed(start.m_rawValue)));

    private static Signed192 ScaleByCenteredAxis(Signed192 value) =>
        // A doubled full-domain coordinate difference plus one normalized-axis
        // component times a nonnegative Fixed64 full length uses at most 98 signed
        // bits. The upper two words are therefore sign extension.
        Signed192.NarrowValue(WideArithmetic.MultiplySigned192(value, DoubleParameterScale));

    private static bool TrySolveUnitQuadratic(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        Signed320 one = Scale320;
        return TrySolveBoundedQuadratic(
            coefficient,
            projection,
            constant,
            new RationalBound320(default, one),
            new RationalBound320(one, one),
            Fixed64.One,
            out entry,
            out exit);
    }

}
