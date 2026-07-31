//=======================================================================
// WideOrientedBox.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns exact full-domain reducers for <see cref="FixedOrientedBox"/>.
/// </summary>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct RadialPenetration
    {
        internal readonly WideAxis3 Axis;
        internal readonly bool Negate;
        internal readonly Fixed64 Depth;
        internal readonly bool DepthIsClamped;
        internal readonly Signed576 Rational;
        internal readonly Signed832 RadialNumerator;
        internal readonly Signed576 RadialDenominator;
        internal readonly Signed576 AxisSquared;

        internal RadialPenetration(
            WideAxis3 axis,
            bool negate,
            Fixed64 depth,
            bool depthIsClamped,
            Signed576 rational,
            Signed832 radialNumerator,
            Signed576 radialDenominator,
            Signed576 axisSquared)
        {
            Axis = axis;
            Negate = negate;
            Depth = depth;
            DepthIsClamped = depthIsClamped;
            Rational = rational;
            RadialNumerator = radialNumerator;
            RadialDenominator = radialDenominator;
            AxisSquared = axisSquared;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }


    private readonly struct PointSpanPenetration
    {
        internal readonly WideAxis3 Axis;
        internal readonly bool Negate;
        internal readonly Signed576 ExactOverlap;
        internal readonly Signed576 ExactSquaredAxisLength;
        internal readonly Signed320 ExactCommonDenominator;

        internal PointSpanPenetration(
            WideAxis3 axis,
            bool negate,
            Signed576 exactOverlap,
            Signed576 exactSquaredAxisLength,
            Signed320 exactCommonDenominator)
        {
            Axis = axis;
            Negate = negate;
            ExactOverlap = exactOverlap;
            ExactSquaredAxisLength = exactSquaredAxisLength;
            ExactCommonDenominator = exactCommonDenominator;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }

    #endregion

    internal static void GetAxes(
        FixedQuaternion orientation,
        out Vector3d axisX,
        out Vector3d axisY,
        out Vector3d axisZ)
    {
        WideRationalBasis3d basis = new(orientation);
        axisX = GetAxis(basis.Xx, basis.Xy, basis.Xz, basis.Denominator, false);
        axisY = GetAxis(basis.Yx, basis.Yy, basis.Yz, basis.Denominator, false);
        axisZ = GetAxis(basis.Zx, basis.Zy, basis.Zz, basis.Denominator, false);
    }

    internal static Vector3d GetLocalSupportPoint(
        Vector3d worldDirection,
        FixedQuaternion orientation,
        Vector3d halfExtents)
    {
        WideRationalBasis3d basis = new(orientation);
        GetDirectionProjections(
            worldDirection,
            basis,
            out Signed320 x,
            out Signed320 y,
            out Signed320 z);
        return new Vector3d(
            x.Sign > 0 ? halfExtents.X : -halfExtents.X,
            y.Sign > 0 ? halfExtents.Y : -halfExtents.Y,
            z.Sign > 0 ? halfExtents.Z : -halfExtents.Z);
    }

    internal static FixedBoundBox GetBoundsClippedToDomain(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents)
    {
        WideRationalBasis3d basis = new(orientation);
        Signed576 denominator = ToSigned576(basis.Denominator);
        Signed320 minimumNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(long.MinValue),
            basis.Denominator);
        Signed320 maximumNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(long.MaxValue),
            basis.Denominator);
        Signed320 xExtent = GetExtentNumerator(
            basis.Xx,
            basis.Yx,
            basis.Zx,
            halfExtents);
        Signed320 yExtent = GetExtentNumerator(
            basis.Xy,
            basis.Yy,
            basis.Zy,
            halfExtents);
        Signed320 zExtent = GetExtentNumerator(
            basis.Xz,
            basis.Yz,
            basis.Zz,
            halfExtents);
        return FixedBoundBox.FromMinMax(
            new Vector3d(
                GetClippedEndpoint(
                    center.X, xExtent, basis.Denominator, denominator,
                    minimumNumerator, maximumNumerator, true),
                GetClippedEndpoint(
                    center.Y, yExtent, basis.Denominator, denominator,
                    minimumNumerator, maximumNumerator, true),
                GetClippedEndpoint(
                    center.Z, zExtent, basis.Denominator, denominator,
                    minimumNumerator, maximumNumerator, true)),
            new Vector3d(
                GetClippedEndpoint(
                    center.X, xExtent, basis.Denominator, denominator,
                    minimumNumerator, maximumNumerator, false),
                GetClippedEndpoint(
                    center.Y, yExtent, basis.Denominator, denominator,
                    minimumNumerator, maximumNumerator, false),
                GetClippedEndpoint(
                    center.Z, zExtent, basis.Denominator, denominator,
                    minimumNumerator, maximumNumerator, false)));
    }

    internal static bool Contains(
        Vector3d point,
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents)
    {
        WideRationalBasis3d basis = new(orientation);
        GetPointProjections(
            point,
            center,
            basis,
            out Signed320 x,
            out Signed320 y,
            out Signed320 z);
        return IsWithinExtent(x, halfExtents.X, basis.Denominator)
            && IsWithinExtent(y, halfExtents.Y, basis.Denominator)
            && IsWithinExtent(z, halfExtents.Z, basis.Denominator);
    }

    internal static FixedPointAnchor GetClosestPointAnchor(
        Vector3d point,
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents)
    {
        WideRationalBasis3d basis = new(orientation);
        GetPointProjections(
            point,
            center,
            basis,
            out Signed320 x,
            out Signed320 y,
            out Signed320 z);
        Signed320 xExtent = GetExtentNumerator(halfExtents.X, basis.Denominator);
        Signed320 yExtent = GetExtentNumerator(halfExtents.Y, basis.Denominator);
        Signed320 zExtent = GetExtentNumerator(halfExtents.Z, basis.Denominator);
        bool outsideX = WideArithmetic.CompareMagnitude(x, xExtent) > 0;
        bool outsideY = WideArithmetic.CompareMagnitude(y, yExtent) > 0;
        bool outsideZ = WideArithmetic.CompareMagnitude(z, zExtent) > 0;

        Signed320 localX = GetClampedNumerator(x, xExtent, outsideX);
        Signed320 localY = GetClampedNumerator(y, yExtent, outsideY);
        Signed320 localZ = GetClampedNumerator(z, zExtent, outsideZ);
        if (!(outsideX || outsideY || outsideZ))
        {
            Signed320 xClearance = WideArithmetic.SubtractSigned320(xExtent, GetMagnitude(x));
            Signed320 yClearance = WideArithmetic.SubtractSigned320(yExtent, GetMagnitude(y));
            Signed320 zClearance = WideArithmetic.SubtractSigned320(zExtent, GetMagnitude(z));
            if (WideArithmetic.CompareMagnitude(xClearance, yClearance) <= 0
                && WideArithmetic.CompareMagnitude(xClearance, zClearance) <= 0)
            {
                localX = GetSignedExtent(x, xExtent);
            }
            else if (WideArithmetic.CompareMagnitude(yClearance, zClearance) <= 0)
            {
                localY = GetSignedExtent(y, yExtent);
            }
            else
            {
                localZ = GetSignedExtent(z, zExtent);
            }
        }

        return new FixedPointAnchor(
            center,
            orientation,
            GetRationalLocalPoint(
                basis,
                localX,
                localY,
                localZ));
    }

    internal static Vector3d GetNearestFaceNormal(
        Vector3d point,
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents)
    {
        WideRationalBasis3d basis = new(orientation);
        GetPointProjections(
            point,
            center,
            basis,
            out Signed320 x,
            out Signed320 y,
            out Signed320 z);
        Signed320 xDistance = WideArithmetic.SubtractSigned320(
            GetExtentNumerator(halfExtents.X, basis.Denominator),
            GetMagnitude(x));
        Signed320 yDistance = WideArithmetic.SubtractSigned320(
            GetExtentNumerator(halfExtents.Y, basis.Denominator),
            GetMagnitude(y));
        Signed320 zDistance = WideArithmetic.SubtractSigned320(
            GetExtentNumerator(halfExtents.Z, basis.Denominator),
            GetMagnitude(z));

        if (xDistance.Sign < 0)
            return GetAxis(basis.Xx, basis.Xy, basis.Xz, basis.Denominator, x.Sign < 0);
        if (yDistance.Sign < 0)
            return GetAxis(basis.Yx, basis.Yy, basis.Yz, basis.Denominator, y.Sign < 0);
        if (zDistance.Sign < 0)
            return GetAxis(basis.Zx, basis.Zy, basis.Zz, basis.Denominator, z.Sign < 0);
        if (WideArithmetic.CompareMagnitude(xDistance, yDistance) <= 0
            && WideArithmetic.CompareMagnitude(xDistance, zDistance) <= 0)
        {
            return GetAxis(basis.Xx, basis.Xy, basis.Xz, basis.Denominator, x.Sign < 0);
        }
        if (WideArithmetic.CompareMagnitude(yDistance, zDistance) <= 0)
            return GetAxis(basis.Yx, basis.Yy, basis.Yz, basis.Denominator, y.Sign < 0);
        return GetAxis(basis.Zx, basis.Zy, basis.Zz, basis.Denominator, z.Sign < 0);
    }

    internal static bool TryMaterializeLocalPoint(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d localPoint,
        out Vector3d worldPoint)
    {
        WideRationalBasis3d basis = new(orientation);
        Signed576 denominator = ToSigned576(basis.Denominator);
        bool representable = TryMaterializeCoordinate(
            center.X,
            basis.Xx,
            basis.Yx,
            basis.Zx,
            basis.Denominator,
            denominator,
            localPoint,
            out Fixed64 x)
            & TryMaterializeCoordinate(
                center.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                denominator,
                localPoint,
                out Fixed64 y)
            & TryMaterializeCoordinate(
                center.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                denominator,
                localPoint,
                out Fixed64 z);
        if (!representable)
        {
            worldPoint = default;
            return false;
        }

        worldPoint = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryMaterializeLocalPoint(
        Vector3d firstOrigin,
        Vector3d secondOrigin,
        FixedQuaternion orientation,
        Vector3d localPoint,
        out Vector3d worldPoint)
    {
        WideRationalBasis3d basis = new(orientation);
        Signed576 denominator = ToSigned576(basis.Denominator);
        bool representable = TryMaterializeCoordinate(
                firstOrigin.X,
                secondOrigin.X,
                basis.Xx,
                basis.Yx,
                basis.Zx,
                basis.Denominator,
                denominator,
                localPoint,
                out Fixed64 x)
            & TryMaterializeCoordinate(
                firstOrigin.Y,
                secondOrigin.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                denominator,
                localPoint,
                out Fixed64 y)
            & TryMaterializeCoordinate(
                firstOrigin.Z,
                secondOrigin.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                denominator,
                localPoint,
                out Fixed64 z);
        if (!representable)
        {
            worldPoint = default;
            return false;
        }

        worldPoint = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryTransformLocalOffset(
        FixedQuaternion orientation,
        Vector3d localOffset,
        out Vector3d worldOffset) =>
        TryMaterializeLocalPoint(
            Vector3d.Zero,
            orientation,
            localOffset,
            out worldOffset);

    internal static bool TryGetRelativeOffset(
        FixedQuaternion orientation,
        Vector3d firstOrigin,
        Vector3d firstOffset,
        Vector3d secondOrigin,
        Vector3d secondLocalPoint,
        out Vector3d result)
    {
        WideRationalBasis3d basis = new(orientation);
        Signed576 denominator = ToSigned576(basis.Denominator);
        bool representable = TryGetRelativeOffsetCoordinate(
                firstOrigin.X,
                firstOffset.X,
                secondOrigin.X,
                basis.Xx,
                basis.Yx,
                basis.Zx,
                basis.Denominator,
                denominator,
                secondLocalPoint,
                out Fixed64 x)
            & TryGetRelativeOffsetCoordinate(
                firstOrigin.Y,
                firstOffset.Y,
                secondOrigin.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                denominator,
                secondLocalPoint,
                out Fixed64 y)
            & TryGetRelativeOffsetCoordinate(
                firstOrigin.Z,
                firstOffset.Z,
                secondOrigin.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                denominator,
                secondLocalPoint,
                out Fixed64 z);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryGetSupportOffset(
        Vector3d worldDirection,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        out Vector3d centerOffset)
    {
        WideRationalBasis3d basis = new(orientation);
        Vector3d localSupport = GetLocalSupportPoint(
            worldDirection,
            orientation,
            halfExtents);
        return TryMaterializeRationalOffset(
            basis,
            localSupport,
            out centerOffset);
    }

    internal static bool TryGetSupportDifference(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d otherOrigin,
        Vector3d otherOriginSupportOffset,
        Vector3d worldDirection,
        out Vector3d difference)
    {
        WideRationalBasis3d basis = new(orientation);
        Vector3d localSupport = GetLocalSupportPoint(
            worldDirection,
            orientation,
            halfExtents);
        Signed576 denominator = ToSigned576(basis.Denominator);
        bool representable = TryMaterializeSupportDifferenceCoordinate(
            center.X,
            otherOrigin.X,
            otherOriginSupportOffset.X,
            basis.Xx,
            basis.Yx,
            basis.Zx,
            basis.Denominator,
            denominator,
            localSupport,
            out Fixed64 x)
            & TryMaterializeSupportDifferenceCoordinate(
                center.Y,
                otherOrigin.Y,
                otherOriginSupportOffset.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                denominator,
                localSupport,
                out Fixed64 y)
            & TryMaterializeSupportDifferenceCoordinate(
                center.Z,
                otherOrigin.Z,
                otherOriginSupportOffset.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                denominator,
                localSupport,
                out Fixed64 z);
        if (!representable)
        {
            difference = default;
            return false;
        }

        difference = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryGetSupportDifference(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d otherCenter,
        FixedQuaternion otherOrientation,
        Vector3d otherHalfExtents,
        Vector3d worldDirection,
        out Vector3d difference)
    {
        WideRationalBasis3d basis = new(orientation);
        WideRationalBasis3d otherBasis = new(otherOrientation);
        Vector3d localSupport = GetLocalSupportPoint(
            worldDirection,
            orientation,
            halfExtents);
        Vector3d otherLocalSupport = GetLocalSupportPoint(
            -worldDirection,
            otherOrientation,
            otherHalfExtents);
        Signed320 denominator = WideArithmetic.MultiplySigned192(
            basis.Denominator,
            otherBasis.Denominator);
        Signed576 denominatorWide = Signed576.ExtendValue(
            denominator);
        bool representable = TryMaterializeBoxSupportDifferenceCoordinate(
                center.X,
                otherCenter.X,
                basis.Xx,
                basis.Yx,
                basis.Zx,
                basis.Denominator,
                localSupport,
                otherBasis.Xx,
                otherBasis.Yx,
                otherBasis.Zx,
                otherBasis.Denominator,
                otherLocalSupport,
                denominator,
                denominatorWide,
                out Fixed64 x)
            & TryMaterializeBoxSupportDifferenceCoordinate(
                center.Y,
                otherCenter.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                localSupport,
                otherBasis.Xy,
                otherBasis.Yy,
                otherBasis.Zy,
                otherBasis.Denominator,
                otherLocalSupport,
                denominator,
                denominatorWide,
                out Fixed64 y)
            & TryMaterializeBoxSupportDifferenceCoordinate(
                center.Z,
                otherCenter.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                localSupport,
                otherBasis.Xz,
                otherBasis.Yz,
                otherBasis.Zz,
                otherBasis.Denominator,
                otherLocalSupport,
                denominator,
                denominatorWide,
                out Fixed64 z);
        if (!representable)
        {
            difference = default;
            return false;
        }

        difference = new Vector3d(x, y, z);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetDirectionProjections(
        Vector3d direction,
        WideRationalBasis3d basis,
        out Signed320 x,
        out Signed320 y,
        out Signed320 z)
    {
        Signed192 directionX = Signed192.Raw(direction.X);
        Signed192 directionY = Signed192.Raw(direction.Y);
        Signed192 directionZ = Signed192.Raw(direction.Z);
        x = WideArithmetic.GetDotProduct3D(directionX, directionY, directionZ, basis.Xx, basis.Xy, basis.Xz);
        y = WideArithmetic.GetDotProduct3D(directionX, directionY, directionZ, basis.Yx, basis.Yy, basis.Yz);
        z = WideArithmetic.GetDotProduct3D(directionX, directionY, directionZ, basis.Zx, basis.Zy, basis.Zz);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetPointProjections(
        Vector3d point,
        Vector3d center,
        WideRationalBasis3d basis,
        out Signed320 x,
        out Signed320 y,
        out Signed320 z)
    {
        Signed192 differenceX = WideArithmetic.SubtractSigned192(Signed192.Raw(point.X), Signed192.Raw(center.X));
        Signed192 differenceY = WideArithmetic.SubtractSigned192(Signed192.Raw(point.Y), Signed192.Raw(center.Y));
        Signed192 differenceZ = WideArithmetic.SubtractSigned192(Signed192.Raw(point.Z), Signed192.Raw(center.Z));
        x = WideArithmetic.GetDotProduct3D(differenceX, differenceY, differenceZ, basis.Xx, basis.Xy, basis.Xz);
        y = WideArithmetic.GetDotProduct3D(differenceX, differenceY, differenceZ, basis.Yx, basis.Yy, basis.Yz);
        z = WideArithmetic.GetDotProduct3D(differenceX, differenceY, differenceZ, basis.Zx, basis.Zy, basis.Zz);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsWithinExtent(
        Signed320 projection,
        Fixed64 extent,
        Signed192 denominator) =>
        WideArithmetic.CompareMagnitude(
            projection,
            GetExtentNumerator(extent, denominator)) <= 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed320 GetClampedNumerator(
        Signed320 projection,
        Signed320 extent,
        bool outside) =>
        outside ? GetSignedExtent(projection, extent) : projection;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed320 GetSignedExtent(Signed320 projection, Signed320 extent) =>
        projection.Sign < 0
            ? WideArithmetic.SubtractSigned320(default, extent)
            : extent;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed320 GetExtentNumerator(Fixed64 extent, Signed192 denominator) =>
        WideArithmetic.MultiplySigned192(Signed192.Raw(extent), denominator);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed576 GetMagnitude(Signed576 value) =>
        value.Sign < 0
            ? WideArithmetic.SubtractSigned576(default, value)
            : value;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed320 GetMagnitude(Signed320 value) =>
        value.Sign < 0
            ? WideArithmetic.SubtractSigned320(default, value)
            : value;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 GetMagnitude(Signed192 value) =>
        value.Sign < 0
            ? WideArithmetic.SubtractSigned192(default, value)
            : value;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed320 GetExtentNumerator(
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Vector3d halfExtents) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(GetMagnitude(axisX), Signed192.Raw(halfExtents.X)),
                WideArithmetic.MultiplySigned192(GetMagnitude(axisY), Signed192.Raw(halfExtents.Y))),
            WideArithmetic.MultiplySigned192(GetMagnitude(axisZ), Signed192.Raw(halfExtents.Z)));

    private static Fixed64 GetClippedEndpoint(
        Fixed64 center,
        Signed320 extentNumerator,
        Signed192 denominator,
        Signed576 denominatorWide,
        Signed320 minimumNumerator,
        Signed320 maximumNumerator,
        bool minimum)
    {
        Signed320 centerNumerator = WideArithmetic.MultiplySigned192(Signed192.Raw(center), denominator);
        Signed320 endpointNumerator = minimum
            ? WideArithmetic.SubtractSigned320(centerNumerator, extentNumerator)
            : WideArithmetic.AddSigned320(centerNumerator, extentNumerator);
        if (CompareSigned(endpointNumerator, minimumNumerator) <= 0)
            return Fixed64.MinValue;
        if (CompareSigned(endpointNumerator, maximumNumerator) >= 0)
            return Fixed64.MaxValue;

        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(endpointNumerator),
            denominatorWide,
            out Fixed64 endpoint);
        Signed320 roundedNumerator = WideArithmetic.MultiplySigned192(Signed192.Raw(endpoint), denominator);
        int comparison = CompareSigned(roundedNumerator, endpointNumerator);
        if (minimum && comparison > 0)
            return Fixed64.FromRaw(endpoint.m_rawValue - 1L);
        if (!minimum && comparison < 0)
            return Fixed64.FromRaw(endpoint.m_rawValue + 1L);
        return endpoint;
    }

    private static Vector3d GetAxis(
        Signed192 x,
        Signed192 y,
        Signed192 z,
        Signed192 denominator,
        bool negate)
    {
        if (negate)
        {
            x = WideArithmetic.SubtractSigned192(default, x);
            y = WideArithmetic.SubtractSigned192(default, y);
            z = WideArithmetic.SubtractSigned192(default, z);
        }

        Signed576 denominatorWide = ToSigned576(denominator);
        Signed192 scale = Signed192.Signed(1L << FixedMath.SHIFT_AMOUNT_I);
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(WideArithmetic.MultiplySigned192(x, scale)),
            denominatorWide,
            out Fixed64 roundedX);
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(WideArithmetic.MultiplySigned192(y, scale)),
            denominatorWide,
            out Fixed64 roundedY);
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(WideArithmetic.MultiplySigned192(z, scale)),
            denominatorWide,
            out Fixed64 roundedZ);
        return new Vector3d(roundedX, roundedY, roundedZ);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int CompareSigned(Signed320 left, Signed320 right)
    {
        int leftSign = left.Sign;
        int rightSign = right.Sign;
        if (leftSign != rightSign)
            return leftSign < rightSign ? -1 : 1;
        return WideArithmetic.SubtractSigned320(left, right).Sign;
    }

    private static int CompareSigned(Signed576 left, Signed576 right) =>
        WideArithmetic.SubtractSigned576(left, right).Sign;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed576 ToSigned576(Signed192 value) =>
        Signed576.ExtendValue(Signed320.ExtendValue(value));

}
