//=======================================================================
// ScaledRigidFrame3d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Retains scaled source points and a separately rotated placement without
/// rounding either scale products, reference subtraction, or frame origins.
/// </summary>
/// <remarks>
/// The exact point is T + Rp(offset * ownerScale) +
/// Rg((source - reference) * ownerScale * partScale). Scale factors may be
/// signed or zero; rotations must be normalized. All inputs use their complete
/// scalar domains. Only final Q32.32 materialization can fail representability.
/// </remarks>
internal readonly struct ScaledRigidFrame3d : IForwardHullFrame3d
{
    private static readonly Signed192 ScaleDenominator = new(0UL, 1UL, 0UL);
    private readonly Vector3d _reference;
    private readonly Vector3d _translation;
    private readonly Signed192 _scaleX, _scaleY, _scaleZ;
    private readonly Signed192 _offsetX, _offsetY, _offsetZ;
    private readonly Signed320 _translationX, _translationY, _translationZ;
    private readonly Signed192 _placementDenominator;
    private readonly bool _sameRotation, _identityUnit, _isRigid;

    internal readonly WideRationalBasis3d GeometryBasis;
    internal readonly FixedQuaternion GeometryRotation;
    internal readonly Signed192 RotationDenominator;
    internal readonly Signed320 Denominator;

    internal bool IsRigid => _isRigid;

    internal Vector3d RigidOrigin => _translation;
    internal FixedQuaternion Rotation => GeometryRotation;
    internal bool HasLocalPlacement => !(_offsetX.IsZero & _offsetY.IsZero & _offsetZ.IsZero);

    Signed192 IForwardHullFrame3d.RotationDenominator => RotationDenominator;
    Signed320 IForwardHullFrame3d.Denominator => Denominator;
    RationalPoint3d IForwardHullFrame3d.GetPoint(Vector3d point) => GetPoint(point);
    WideDirection3d IForwardHullFrame3d.GetFaceNormal(in FixedTriangle triangle) =>
        WideTriangleRelations.GetScaledTriangleLocalNormal(triangle, this).Transform(GeometryBasis);
    WideDirection3d IForwardHullFrame3d.GetEdgeDirection(Vector3d first, Vector3d second) =>
        WideDirection3d.GetLocalEdge(this, first, second).Transform(GeometryBasis);
    WideDirection3d IForwardHullFrame3d.GetInwardAxis(in FixedTriangle triangle, Vector3d first, Vector3d second) =>
        WideDirection3d.Cross(WideTriangleRelations.GetScaledTriangleLocalNormal(triangle, this),
            WideDirection3d.GetLocalEdge(this, first, second)).Transform(GeometryBasis);
    bool IForwardHullFrame3d.TryGetTriangleHalfPlanes(in FixedTriangle triangle, in RationalPoint3d point,
        Span<Signed832> values) => WideTriangleRelations.GetScaledTriangleHalfPlanes(triangle, this, point, values);

    internal static ScaledRigidFrame3d Rigid(Vector3d origin, FixedQuaternion rotation) =>
        new(origin, rotation, Vector3d.Zero, Vector3d.One, rotation, Vector3d.Zero, Vector3d.One);

    internal ScaledRigidFrame3d(
        Vector3d translation,
        FixedQuaternion placementRotation,
        Vector3d offset,
        Vector3d ownerScale,
        FixedQuaternion geometryRotation,
        Vector3d reference,
        Vector3d partScale)
    {
        this = default;
        if (!placementRotation.IsNormalized())
            throw new ArgumentException("The placement rotation must be normalized.", nameof(placementRotation));
        if (!geometryRotation.IsNormalized())
            throw new ArgumentException("The geometry rotation must be normalized.", nameof(geometryRotation));

        _reference = reference;
        _translation = translation;
        _sameRotation = placementRotation == geometryRotation;
        _identityUnit = placementRotation == FixedQuaternion.Identity
            && geometryRotation == FixedQuaternion.Identity
            && ownerScale == Vector3d.One && partScale == Vector3d.One;
        GeometryBasis = new WideRationalBasis3d(geometryRotation);
        GeometryRotation = geometryRotation;
        _scaleX = Fixed64.GetExactRawProduct(ownerScale.X, partScale.X);
        _scaleY = Fixed64.GetExactRawProduct(ownerScale.Y, partScale.Y);
        _scaleZ = Fixed64.GetExactRawProduct(ownerScale.Z, partScale.Z);
        // Placement consumes owner scale only. Its Q denominator is lifted
        // to the same Q^2 denominator as the centered three-factor product.
        _offsetX = WideArithmetic.ShiftLeftSigned192By32(Fixed64.GetExactRawProduct(offset.X, ownerScale.X));
        _offsetY = WideArithmetic.ShiftLeftSigned192By32(Fixed64.GetExactRawProduct(offset.Y, ownerScale.Y));
        _offsetZ = WideArithmetic.ShiftLeftSigned192By32(Fixed64.GetExactRawProduct(offset.Z, ownerScale.Z));

        // Frame inputs are immutable. Classify their exact rigid mapping once,
        // rather than repeating scale/reference products for every support query.
        _isRigid = _sameRotation && _scaleX.Equals(ScaleDenominator)
            && _scaleY.Equals(ScaleDenominator) && _scaleZ.Equals(ScaleDenominator)
            && _offsetX.Equals(ScaleReference(_reference.X))
            && _offsetY.Equals(ScaleReference(_reference.Y))
            && _offsetZ.Equals(ScaleReference(_reference.Z));

        WideRationalBasis3d placementBasis = _sameRotation ? GeometryBasis : new(placementRotation);
        _placementDenominator = _sameRotation ? Signed192.Signed(1L) : placementBasis.Denominator;
        RotationDenominator = Signed192.NarrowProven(
            WideArithmetic.MultiplySigned192(GeometryBasis.Denominator, _placementDenominator));
        Denominator = WideArithmetic.MultiplySigned192(RotationDenominator, ScaleDenominator);
        _translationX = GetTranslation(translation.X, placementBasis.Xx, placementBasis.Yx, placementBasis.Zx);
        _translationY = GetTranslation(translation.Y, placementBasis.Xy, placementBasis.Yy, placementBasis.Zy);
        _translationZ = GetTranslation(translation.Z, placementBasis.Xz, placementBasis.Yz, placementBasis.Zz);
    }

    /// <summary>
    /// Returns centered scaled raw-coordinate numerators over the shared Q^2
    /// denominator. A full-domain difference times two scalar factors fits 192 bits.
    /// </summary>
    internal void GetScaledLocalPointNumerators(Vector3d source, out Signed192 x, out Signed192 y, out Signed192 z)
    {
        x = GetScaledDifference(source.X, _reference.X, _scaleX);
        y = GetScaledDifference(source.Y, _reference.Y, _scaleY);
        z = GetScaledDifference(source.Z, _reference.Z, _scaleZ);
    }

    internal RationalPoint3d GetPoint(Vector3d source)
    {
        if (_identityUnit)
        {
            // Preserve the shared Q^2 denominator with shifts, without rotation products.
            return new RationalPoint3d(GetOrdinaryCoordinate(source.X, _reference.X, _offsetX, _translationX),
                GetOrdinaryCoordinate(source.Y, _reference.Y, _offsetY, _translationY),
                GetOrdinaryCoordinate(source.Z, _reference.Z, _offsetZ, _translationZ), Denominator);
        }

        GetScaledLocalPointNumerators(source, out Signed192 x, out Signed192 y, out Signed192 z);
        if (_sameRotation)
        {
            x = WideArithmetic.AddSigned192(x, _offsetX);
            y = WideArithmetic.AddSigned192(y, _offsetY);
            z = WideArithmetic.AddSigned192(z, _offsetZ);
        }
        return new RationalPoint3d(
            GetWorldCoordinate(x, y, z, GeometryBasis.Xx, GeometryBasis.Yx, GeometryBasis.Zx, _translationX),
            GetWorldCoordinate(x, y, z, GeometryBasis.Xy, GeometryBasis.Yy, GeometryBasis.Zy, _translationY),
            GetWorldCoordinate(x, y, z, GeometryBasis.Xz, GeometryBasis.Yz, GeometryBasis.Zz, _translationZ),
            Denominator);
    }

    internal bool TryGetPoint(Vector3d source, out Vector3d point) => GetPoint(source).TryGetPoint(out point);

    internal RationalPoint3d GetScaledLocalPoint(Vector3d scaledLocalPoint)
    {
        Signed192 x = ScaleReference(scaledLocalPoint.X);
        Signed192 y = ScaleReference(scaledLocalPoint.Y);
        Signed192 z = ScaleReference(scaledLocalPoint.Z);
        if (_sameRotation)
        {
            x = WideArithmetic.AddSigned192(x, _offsetX);
            y = WideArithmetic.AddSigned192(y, _offsetY);
            z = WideArithmetic.AddSigned192(z, _offsetZ);
        }
        return new RationalPoint3d(
            GetWorldCoordinate(x, y, z, GeometryBasis.Xx, GeometryBasis.Yx, GeometryBasis.Zx, _translationX),
            GetWorldCoordinate(x, y, z, GeometryBasis.Xy, GeometryBasis.Yy, GeometryBasis.Zy, _translationY),
            GetWorldCoordinate(x, y, z, GeometryBasis.Xz, GeometryBasis.Yz, GeometryBasis.Zz, _translationZ), Denominator);
    }

    internal bool TryGetScaledLocalPointIn(Vector3d worldPoint, out Vector3d scaledLocalPoint)
    {
        GetScaledLocalPointNumeratorsIn(worldPoint, out Signed576 x, out Signed576 y, out Signed576 z,
            out Signed320 denominator);
        Signed576 d = Signed576.ExtendValue(denominator);
        bool representable = Fixed64.TryGetSignedRawRatio(x, d, out Fixed64 px)
            & Fixed64.TryGetSignedRawRatio(y, d, out Fixed64 py)
            & Fixed64.TryGetSignedRawRatio(z, d, out Fixed64 pz);
        scaledLocalPoint = representable ? new Vector3d(px, py, pz) : default;
        return representable;
    }

    internal bool IsScaledLocalPointExact(Vector3d source, Vector3d roundedCentered)
    {
        GetScaledLocalPointNumerators(source, out Signed192 x, out Signed192 y, out Signed192 z);
        return x.Equals(ScaleReference(roundedCentered.X))
            & y.Equals(ScaleReference(roundedCentered.Y))
            & z.Equals(ScaleReference(roundedCentered.Z));
    }

    internal WideAxis3 GetSourceDirectionNumerators(Vector3d worldDirection)
    {
        Signed192 x = Signed192.Raw(worldDirection.X), y = Signed192.Raw(worldDirection.Y), z = Signed192.Raw(worldDirection.Z);
        // Rotation coefficients <65 bits, direction components <=64 and scale
        // products <=127 keep each complete source covector below 258 bits.
        return new WideAxis3(
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
                Rotate(x, y, z, GeometryBasis.Xx, GeometryBasis.Xy, GeometryBasis.Xz), _scaleX)),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
                Rotate(x, y, z, GeometryBasis.Yx, GeometryBasis.Yy, GeometryBasis.Yz), _scaleY)),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
                Rotate(x, y, z, GeometryBasis.Zx, GeometryBasis.Zy, GeometryBasis.Zz), _scaleZ)));
    }

    /// <summary>Transforms an authored endpoint difference, whose components have magnitude below 2^64.</summary>
    internal WideAxis3 GetWorldDirectionNumerators(in WideAxis3 authoredRay)
    {
        // A halfspace direction has no metric scale. Rigid endpoint
        // differences need only the rotation numerator; retaining unit Q^2
        // scale factors needlessly enlarges every downstream fan product.
        if (IsRigid)
            return WideRigidProjection.TransformLocalAxis(GeometryBasis,
                Signed192.NarrowProven(authoredRay.X), Signed192.NarrowProven(authoredRay.Y), Signed192.NarrowProven(authoredRay.Z));
        Signed192 x = Signed192.NarrowValue(WideArithmetic.MultiplySigned320(authoredRay.X, _scaleX));
        Signed192 y = Signed192.NarrowValue(WideArithmetic.MultiplySigned320(authoredRay.Y, _scaleY));
        Signed192 z = Signed192.NarrowValue(WideArithmetic.MultiplySigned320(authoredRay.Z, _scaleZ));
        return new WideAxis3(Rotate(x, y, z, GeometryBasis.Xx, GeometryBasis.Yx, GeometryBasis.Zx),
            Rotate(x, y, z, GeometryBasis.Xy, GeometryBasis.Yy, GeometryBasis.Zy),
            Rotate(x, y, z, GeometryBasis.Xz, GeometryBasis.Yz, GeometryBasis.Zz));
    }

    internal FixedBoundBox GetWorldBounds(Vector3d sourceMin, Vector3d sourceMax)
    {
        if (IsRigid)
            return FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(RigidOrigin, Rotation, sourceMin, sourceMax,
                Vector3d.Zero, FixedQuaternion.Identity);
        Vector3d min = new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue);
        Vector3d max = new(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue);
        for (int corner = 0; corner < 8; corner++)
        {
            RationalPoint3d point = GetPoint(GetCorner(sourceMin, sourceMax, corner));
            Signed576 denominator = Signed576.ExtendValue(point.Denominator);
            IncludeBound(Signed576.ExtendValue(point.X), denominator, ref min.X, ref max.X);
            IncludeBound(Signed576.ExtendValue(point.Y), denominator, ref min.Y, ref max.Y);
            IncludeBound(Signed576.ExtendValue(point.Z), denominator, ref min.Z, ref max.Z);
        }
        return FixedBoundBox.FromMinMax(min, max);
    }

    internal bool TryGetSourceBounds(Vector3d worldMin, Vector3d worldMax, out FixedBoundBox bounds)
    {
        if (_scaleX.IsZero || _scaleY.IsZero || _scaleZ.IsZero)
        {
            bounds = default;
            return false;
        }
        Vector3d min = new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue);
        Vector3d max = new(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue);
        for (int corner = 0; corner < 8; corner++)
        {
            GetSourcePointNumerators(GetCorner(worldMin, worldMax, corner),
                out Signed576 x, out Signed576 y, out Signed576 z,
                out Signed576 dx, out Signed576 dy, out Signed576 dz);
            IncludeBound(x, dx, ref min.X, ref max.X);
            IncludeBound(y, dy, ref min.Y, ref max.Y);
            IncludeBound(z, dz, ref min.Z, ref max.Z);
        }
        bounds = FixedBoundBox.FromMinMax(min, max);
        return true;
    }

    internal bool TryGetBoundsIn(Vector3d sourceMin, Vector3d sourceMax,
        in ScaledRigidFrame3d target, out FixedBoundBox bounds)
    {
        if (target._scaleX.IsZero || target._scaleY.IsZero || target._scaleZ.IsZero)
        {
            bounds = default;
            return false;
        }
        Vector3d min = new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue);
        Vector3d max = new(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue);
        for (int corner = 0; corner < 8; corner++)
        {
            RationalPoint3d world = GetPoint(GetCorner(sourceMin, sourceMax, corner));
            for (int axis = 0; axis < 3; axis++)
            {
                target.GetSourceCoordinateTransform(axis, out Signed576 x, out Signed576 y, out Signed576 z,
                    out Signed576 offset, out Signed576 divisor);
                Signed832 numerator = WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
                    WideArithmetic.MultiplySigned576ToSigned832(x, world.X),
                    WideArithmetic.MultiplySigned576ToSigned832(y, world.Y)),
                    WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(z, world.Z),
                        WideArithmetic.MultiplySigned576ToSigned832(offset, world.Denominator)));
                Signed832 denominator = WideArithmetic.MultiplySigned576ToSigned832(divisor, world.Denominator);
                // Both complete affine ratios retain the source's Q^2 factor.
                // Cancel it exactly before the bounded 576-bit bound owner.
                Signed576 n = RemoveScaleDenominator(numerator), d = RemoveScaleDenominator(denominator);
                min[axis] = FixedMath.Min(min[axis], WideGeometry.GetRationalBoundClippedToDomain(n, d, true));
                max[axis] = FixedMath.Max(max[axis], WideGeometry.GetRationalBoundClippedToDomain(n, d, false));
            }
        }
        bounds = FixedBoundBox.FromMinMax(min, max);
        return true;
    }

    /// <summary>
    /// Inverts this frame without materializing its transformed reference.
    /// Each source coordinate has its own scale denominator; zero scale makes
    /// that coordinate undefined and therefore makes materialization fail.
    /// </summary>
    internal void GetSourcePointNumerators(Vector3d worldPoint,
        out Signed576 x, out Signed576 y, out Signed576 z,
        out Signed576 denominatorX, out Signed576 denominatorY, out Signed576 denominatorZ)
    {
        GetScaledLocalPointNumeratorsIn(worldPoint, out Signed576 inverseX, out Signed576 inverseY,
            out Signed576 inverseZ, out Signed320 rotationDenominator);
        GetSourceCoordinate(inverseX, rotationDenominator,
            _reference.X, _scaleX, out x, out denominatorX);
        GetSourceCoordinate(inverseY, rotationDenominator,
            _reference.Y, _scaleY, out y, out denominatorY);
        GetSourceCoordinate(inverseZ, rotationDenominator,
            _reference.Z, _scaleZ, out z, out denominatorZ);
    }

    internal bool TryGetSourceBoundsFromScaledLocal(Vector3d scaledMin, Vector3d scaledMax, out FixedBoundBox bounds)
    {
        if (_scaleX.IsZero || _scaleY.IsZero || _scaleZ.IsZero) { bounds = default; return false; }
        Vector3d min = default, max = default;
        Signed320 one = Signed320.ExtendValue(Signed192.Signed(1));
        for (int axis = 0; axis < 3; axis++)
        {
            min[axis] = Fixed64.MaxValue; max[axis] = Fixed64.MinValue;
            for (int end = 0; end < 2; end++)
            {
                Fixed64 value = end == 0 ? scaledMin[axis] : scaledMax[axis];
                GetSourceCoordinate(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(value))), one,
                    _reference[axis], GetScale(axis), out Signed576 numerator, out Signed576 denominator);
                min[axis] = FixedMath.Min(min[axis], WideGeometry.GetRationalBoundClippedToDomain(numerator, denominator, true));
                max[axis] = FixedMath.Max(max[axis], WideGeometry.GetRationalBoundClippedToDomain(numerator, denominator, false));
            }
        }
        bounds = FixedBoundBox.FromMinMax(min, max);
        return true;
    }
    internal bool HasSameMapping(in ScaledRigidFrame3d other)
    {
        for (int axis = 0; axis < 3; axis++)
        {
            WideAxis3 first = GeometryBasis.GetAxis(axis), second = other.GeometryBasis.GetAxis(axis);
            Signed192 firstScale = GetScale(axis), secondScale = other.GetScale(axis);
            if (!SameLinearCoordinate(first.X, firstScale, other.GeometryBasis.Denominator,
                    second.X, secondScale, GeometryBasis.Denominator)
                || !SameLinearCoordinate(first.Y, firstScale, other.GeometryBasis.Denominator,
                    second.Y, secondScale, GeometryBasis.Denominator)
                || !SameLinearCoordinate(first.Z, firstScale, other.GeometryBasis.Denominator,
                    second.Z, secondScale, GeometryBasis.Denominator)) return false;
        }
        RationalPoint3d origin = GetPoint(Vector3d.Zero), otherOrigin = other.GetPoint(Vector3d.Zero);
        // Both point denominators contain the same Q^2 factor; cancel it
        // before comparing independently retained placement origins.
        return SameMappedCoordinate(origin.X, other.RotationDenominator, otherOrigin.X, RotationDenominator)
            & SameMappedCoordinate(origin.Y, other.RotationDenominator, otherOrigin.Y, RotationDenominator)
            & SameMappedCoordinate(origin.Z, other.RotationDenominator, otherOrigin.Z, RotationDenominator);
    }

    /// <summary>
    /// Returns one inverse coordinate as (x*worldX + y*worldY + z*worldZ +
    /// offset)/divisor, where world components and the result are raw coordinates.
    /// The axis is in [0,2]; a zero scale produces a zero divisor.
    /// </summary>
    internal void GetSourceCoordinateTransform(int axis, out Signed576 x, out Signed576 y, out Signed576 z,
        out Signed576 offset, out Signed576 divisor)
    {
        WideAxis3 column = GeometryBasis.GetAxis(axis);
        // Every inverse coefficient has a common Q^2 factor; cancel it here
        // so chained geometry does not multiply redundant scale denominators.
        x = WideArithmetic.MultiplySigned320(column.X, Denominator);
        y = WideArithmetic.MultiplySigned320(column.Y, Denominator);
        z = WideArithmetic.MultiplySigned320(column.Z, Denominator);
        divisor = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(WideArithmetic.MultiplySigned192(RotationDenominator, GeometryBasis.Denominator)), GetScale(axis));
        RationalPoint3d origin = GetPoint(_reference);
        Signed576 projectedOrigin = WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(origin.X, column.X), WideArithmetic.MultiplySigned320(origin.Y, column.Y)),
            WideArithmetic.MultiplySigned320(origin.Z, column.Z));
        offset = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned576(divisor, _reference[axis].m_rawValue),
            projectedOrigin);
    }

    private Signed192 GetScale(int axis) => axis switch { 0 => _scaleX, 1 => _scaleY, _ => _scaleZ };

    private static bool SameLinearCoordinate(Signed320 first, Signed192 firstScale, Signed192 otherDenominator,
        Signed320 second, Signed192 secondScale, Signed192 denominator) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(first, firstScale), otherDenominator),
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(second, secondScale), denominator)).IsZero;

    private static bool SameMappedCoordinate(Signed320 first, Signed192 otherDenominator,
        Signed320 second, Signed192 denominator) =>
        WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(first, otherDenominator),
            WideArithmetic.MultiplySigned320(second, denominator)).IsZero;

    private static Vector3d GetCorner(Vector3d min, Vector3d max, int corner) =>
        new((corner & 1) == 0 ? min.X : max.X, (corner & 2) == 0 ? min.Y : max.Y,
            (corner & 4) == 0 ? min.Z : max.Z);

    private static Signed576 RemoveScaleDenominator(Signed832 value) =>
        new(value.Word9, value.Word8, value.Word7, value.Word6, value.Word5,
            value.Word4, value.Word3, value.Word2, value.Word1);

    private static void IncludeBound(Signed576 numerator, Signed576 denominator, ref Fixed64 min, ref Fixed64 max)
    {
        min = FixedMath.Min(min, WideGeometry.GetRationalBoundClippedToDomain(numerator, denominator, minimum: true));
        max = FixedMath.Max(max, WideGeometry.GetRationalBoundClippedToDomain(numerator, denominator, minimum: false));
    }

    private static Signed192 ScaleReference(Fixed64 value) =>
        WideArithmetic.ShiftLeftSigned192By32(WideArithmetic.ShiftLeftSigned192By32(Signed192.Raw(value)));

    private void GetScaledLocalPointNumeratorsIn(Vector3d worldPoint,
        out Signed576 x, out Signed576 y, out Signed576 z, out Signed320 denominator)
    {
        GetPoint(_reference).GetLocalPointNumerators(worldPoint, GeometryRotation,
            out Signed576 inverseX, out Signed576 inverseY, out Signed576 inverseZ, out denominator);
        // The rigid projection gives reference-worldPoint; negate to obtain the
        // displacement from the exact geometry origin to the requested point.
        x = WideArithmetic.SubtractSigned576(default, inverseX);
        y = WideArithmetic.SubtractSigned576(default, inverseY);
        z = WideArithmetic.SubtractSigned576(default, inverseZ);
    }

    private static void GetSourceCoordinate(Signed576 localNumerator, Signed320 localDenominator,
        Fixed64 reference, Signed192 scale, out Signed576 numerator, out Signed576 denominator)
    {
        denominator = WideArithmetic.MultiplySigned320(localDenominator, scale);
        // Three source/target rotations, two scalar factors and reference
        // multiplication keep these inverse numerators below 452 bits.
        numerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(localNumerator, ScaleDenominator),
            WideArithmetic.MultiplySigned576(denominator, reference.m_rawValue));
    }

    private Signed320 GetTranslation(Fixed64 translation, Signed192 axisX, Signed192 axisY, Signed192 axisZ)
    {
        Signed576 result = WideArithmetic.MultiplySigned320(Denominator, Signed192.Raw(translation));
        if (!_sameRotation)
        {
            Signed320 rotated = Rotate(_offsetX, _offsetY, _offsetZ, axisX, axisY, axisZ);
            result = WideArithmetic.AddSigned576(result,
                WideArithmetic.MultiplySigned320(rotated, GeometryBasis.Denominator));
        }
        return Signed320.NarrowValue(result);
    }

    private Signed320 GetWorldCoordinate(Signed192 x, Signed192 y, Signed192 z,
        Signed192 axisX, Signed192 axisY, Signed192 axisZ, Signed320 translation)
    {
        Signed576 result = WideArithmetic.AddSigned576(Signed576.ExtendValue(translation),
            WideArithmetic.MultiplySigned320(Rotate(x, y, z, axisX, axisY, axisZ), _placementDenominator));
        // Normalized quaternion D < Q^2 + 258Q < (17/16)Q^2.
        // The geometry numerator vector has norm < sqrt(3)*2^190.
        // Two basis denominators give magnitude < (7/4)*(17/16)^2*2^318,
        // leaving more than 2^312 of headroom below the signed 320-bit limit.
        // Full-domain placement and translation together are below 2^290.
        return Signed320.NarrowValue(result);
    }

    private static Signed192 GetScaledDifference(Fixed64 source, Fixed64 reference, Signed192 scale) =>
        Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            WideArithmetic.Difference(source, reference), scale));

    private static Signed320 GetOrdinaryCoordinate(Fixed64 source, Fixed64 reference, Signed192 offset, Signed320 translation) =>
        WideArithmetic.AddSigned320(translation, Signed320.ExtendValue(
            WideArithmetic.ShiftLeftSigned192By32(WideArithmetic.ShiftLeftSigned192By32(
                WideArithmetic.AddSigned192(WideArithmetic.Difference(source, reference),
                    Signed192.Signed(unchecked((long)offset.Middle)))))));

    private static Signed320 Rotate(Signed192 x, Signed192 y, Signed192 z,
        Signed192 axisX, Signed192 axisY, Signed192 axisZ) =>
        WideArithmetic.AddSigned320(WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(x, axisX), WideArithmetic.MultiplySigned192(y, axisY)),
            WideArithmetic.MultiplySigned192(z, axisZ));
}
