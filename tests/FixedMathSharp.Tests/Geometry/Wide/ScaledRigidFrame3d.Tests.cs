//=======================================================================
// ScaledRigidFrame3d.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Linq;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class ScaledRigidFrame3dTests
{
    private const long Q = 1L << 32;
    private static readonly BigInteger Q2 = BigInteger.One << 64;
    private static readonly FixedQuaternion Rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)(-23), (Fixed64)45);
    private static readonly FixedQuaternion OtherRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)(-31), (Fixed64)11, (Fixed64)67);

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void InverseMaterialization_ShouldRejectAnUnrepresentableRelativeCoordinate(int axis, bool maximumOrigin)
    {
        Vector3d origin = Vector3d.Zero, query = Vector3d.Zero;
        origin[axis] = maximumOrigin ? Fixed64.MaxValue : Fixed64.MinValue;
        query[axis] = maximumOrigin ? Fixed64.MinValue : Fixed64.MaxValue;
        var frame = ScaledRigidFrame3d.Rigid(origin, FixedQuaternion.Identity);

        // Both absolute points fit, but their exact relative difference spans
        // the entire scalar domain and cannot be returned as one Fixed64.
        Assert.False(frame.TryGetScaledLocalPointIn(query, out Vector3d local));
        Assert.Equal(Vector3d.Zero, local);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void SingularTargetBounds_ShouldRejectEachMissingAxis(int axis)
    {
        Vector3d scale = Vector3d.One; scale[axis] = Fixed64.Zero;
        var source = ScaledRigidFrame3d.Rigid(Vector3d.Zero, Rotation);
        var target = new ScaledRigidFrame3d(Vector3d.Zero, OtherRotation, Vector3d.Zero, scale,
            OtherRotation, Vector3d.Zero, Vector3d.One);
        Assert.False(source.TryGetBoundsIn(-Vector3d.One, Vector3d.One, target, out _));
    }

    [Fact]
    public void ExactProjection_ShouldKeepSubRawOrderingAcrossIndependentDenominators()
    {
        var firstFrame = new ScaledRigidFrame3d(Vector3d.Zero, Rotation, Vector3d.Zero,
            Raw(2863311531, Q, Q), Rotation, Vector3d.Zero, Vector3d.One);
        var secondFrame = new ScaledRigidFrame3d(Vector3d.Zero, OtherRotation, Vector3d.Zero,
            Raw(Q, 2576980378, Q), Rotation, Vector3d.Zero, Vector3d.One);
        RationalPoint3d first = firstFrame.GetPoint(Raw(1, 0, 0)), second = secondFrame.GetPoint(Raw(1, 0, 0));
        Vector3d direction = Rotation.Rotate(Vector3d.Right);
        BigInteger[] a = Coordinates(first), b = Coordinates(second);
        BigInteger projection = 0;
        for (int axis = 0; axis < 3; axis++)
            projection += (a[axis] * Big(second.Denominator) - b[axis] * Big(first.Denominator)) * direction[axis].m_rawValue;
        Assert.Equal(projection.Sign, RationalPoint3d.CompareProjection(first, second, direction));
        Assert.Equal(-projection.Sign, RationalPoint3d.CompareProjection(second, first, direction));
        Assert.Equal(0, RationalPoint3d.CompareProjection(first, first, direction));
    }

    [Fact]
    public void SharedAuthoredVertex_ShouldSurviveIndependentCentersScaleAndRotation()
    {
        Vector3d source = Raw(3865470566, 0, -4 * Q);
        Vector3d[] centers = { Raw(-6657199309, 0, 0), Raw(10522669875, 0, 0), Raw(12670153523, 0, 0) };
        foreach (FixedQuaternion rotation in new[] { FixedQuaternion.Identity, Rotation })
        foreach (Vector3d owner in new[] { Vector3d.One, Raw(2863311531, Q, 2576980378) })
        {
            RationalPoint3d[] points = centers.Select(center => Check(source, center, center, Vector3d.Zero,
                rotation, rotation, owner, Vector3d.One)).ToArray();
            for (int index = 1; index < points.Length; index++)
            {
                Assert.Equal(points[0].X, points[index].X);
                Assert.Equal(points[0].Y, points[index].Y);
                Assert.Equal(points[0].Z, points[index].Z);
                Assert.Equal(points[0].Denominator, points[index].Denominator);
            }
        }
    }

    [Fact]
    public void FractionalNonuniformScale_ShouldPreserveExactPlanarityBeforeMaterialization()
    {
        const long center = 3865470566;
        Vector3d[] vertices = { Raw(center - Q / 2, center - Q / 2, 0), Raw(center, center, 0),
            Raw(center + Q / 2, center + Q / 2, 0), Raw(center, center, Q) };
        Vector3d scale = Raw(2863311531, 2576980378, Q), reference = Raw(center, center, 0);
        RationalPoint3d[] exact = vertices.Select(v => Check(v, reference, reference, Vector3d.Zero,
            Rotation, Rotation, scale, Vector3d.One)).ToArray();
        BigInteger[] a = Subtract(Coordinates(exact[1]), Coordinates(exact[0]));
        BigInteger[] b = Subtract(Coordinates(exact[2]), Coordinates(exact[0]));
        BigInteger[] c = Subtract(Coordinates(exact[3]), Coordinates(exact[0]));
        Assert.All(Cross(a, b), value => Assert.Equal(BigInteger.Zero, value));
        Assert.Equal(BigInteger.Zero, Dot(Cross(a, c), b));
        // Rounding authored scaled vertices separately would introduce this actual kink.
        BigInteger[] x = vertices.Take(3).Select(v => Round((BigInteger)v.X.m_rawValue * scale.X.m_rawValue, Q)).ToArray();
        BigInteger[] y = vertices.Take(3).Select(v => Round((BigInteger)v.Y.m_rawValue * scale.Y.m_rawValue, Q)).ToArray();
        Assert.Equal(new BigInteger(1288490189), (x[1] - x[0]) * (y[2] - y[0]) - (y[1] - y[0]) * (x[2] - x[0]));
    }

    [Fact]
    public void Placement_ShouldUseOwnerScaleOnlyAndRetainSeparateCompoundRotations()
    {
        Vector3d source = Raw(3 * Q + 1, -2 * Q + 3, Q / 3), reference = Raw(Q, -Q, 0);
        Vector3d owner = Raw(2863311531, 2576980378, Q / 2), part = Raw(3 * Q, Q / 3, 2 * Q);
        foreach (FixedQuaternion placement in new[] { FixedQuaternion.Identity, Rotation })
        foreach (FixedQuaternion geometry in new[] { FixedQuaternion.Identity, Rotation, OtherRotation })
            Check(source, reference, Raw(-Q, 2 * Q, Q / 5), Raw(5 * Q, -3 * Q, Q),
                placement, geometry, owner, part);

        // At the source reference the part scale disappears, while the scaled offset remains.
        RationalPoint3d first = Check(reference, reference, source, Vector3d.Zero, Rotation, OtherRotation, owner, part);
        RationalPoint3d second = Check(reference, reference, source, Vector3d.Zero, Rotation, OtherRotation, owner, Vector3d.One);
        Assert.Equal(first.X, second.X);
        Assert.Equal(first.Y, second.Y);
        Assert.Equal(first.Z, second.Z);
        Assert.Equal(first.Denominator, second.Denominator);
    }

    [Fact]
    public void LargeAuthoredCoordinates_ShouldCancelBeforeScaledScalarMaterialization()
    {
        long center = long.MaxValue - 8 * Q;
        RationalPoint3d point = Check(Raw(center + Q, 0, 0), Raw(center, 0, 0), Vector3d.Zero, Vector3d.Zero,
            FixedQuaternion.Identity, FixedQuaternion.Identity, Raw(8 * Q, Q, Q), Vector3d.One);
        Assert.True((BigInteger)(center + Q) * 8 > long.MaxValue);
        Assert.True(point.TryGetPoint(out Vector3d materialized));
        Assert.Equal(Raw(8 * Q, 0, 0), materialized);
    }

    [Fact]
    public void FullDomainOriginsAndFactors_ShouldMatchIndependentArithmeticWithoutIntermediateRejection()
    {
        foreach (long source in new[] { long.MinValue, long.MaxValue })
        foreach (long reference in new[] { long.MinValue, long.MaxValue })
        foreach (FixedQuaternion geometry in new[] { FixedQuaternion.Identity, Rotation, OtherRotation })
            Check(Raw(source, reference, source), Raw(reference, source, reference), Raw(source, source, reference),
                Raw(long.MaxValue, long.MinValue, 0), Rotation, geometry,
                Raw(long.MaxValue, long.MinValue, long.MaxValue), Raw(long.MinValue, long.MaxValue, long.MinValue));

        foreach (int axis in new[] { 0, 1, 2 })
        foreach (long origin in new[] { long.MinValue, long.MaxValue })
        foreach (long displacement in new[] { -Q, 0, Q })
        {
            Vector3d placement = Vector3d.Zero, local = Vector3d.Zero;
            placement[axis] = Fixed64.FromRaw(origin);
            local[axis] = Fixed64.FromRaw(displacement);
            Check(local, Vector3d.Zero, Vector3d.Zero, placement,
                FixedQuaternion.Identity, FixedQuaternion.Identity, Vector3d.One, Vector3d.One);
        }
        Check(Raw(Q, -Q, 2 * Q), Raw(-Q, Q, 0), Raw(Q, Q, Q), Vector3d.Zero,
            Rotation, OtherRotation, Vector3d.Zero, Raw(-Q, 2 * Q, 0));
        Assert.False(default(RationalPoint3d).TryGetPoint(out Vector3d absent));
        Assert.Equal(Vector3d.Zero, absent);
    }

    [Theory]
    [InlineData(-3L, -2L)]
    [InlineData(-1L, 0L)]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    public void Materialization_ShouldRoundTheFinalWorldCoordinateOnceToNearestEven(long sourceRaw, long expectedRaw)
    {
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Raw(Q / 2, Q, Q), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Assert.True(frame.TryGetPoint(Raw(sourceRaw, 0, 0), out Vector3d point));
        Assert.Equal(expectedRaw, point.X.m_rawValue);
        var translated = new ScaledRigidFrame3d(Raw(1, 0, 0), FixedQuaternion.Identity, Vector3d.Zero,
            Raw(Q / 2, Q, Q), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Assert.True(translated.TryGetPoint(Raw(sourceRaw, 0, 0), out Vector3d translatedPoint));
        Assert.Equal((long)Round(2 + sourceRaw, 2), translatedPoint.X.m_rawValue);
    }

    [Fact]
    public void InvalidRotations_ShouldFailAtTheirOwnFrameBoundary()
    {
        var invalid = new FixedQuaternion(Fixed64.One, Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        Assert.Throws<ArgumentException>(() => new ScaledRigidFrame3d(Vector3d.Zero, invalid, Vector3d.Zero,
            Vector3d.One, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One));
        Assert.Throws<ArgumentException>(() => new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Vector3d.One, invalid, Vector3d.Zero, Vector3d.One));
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, Rotation, Vector3d.Zero,
            Vector3d.One, Rotation, Vector3d.Zero, Vector3d.One);
        RationalPoint3d point = frame.GetPoint(Vector3d.Zero);
        Assert.Throws<ArgumentException>(() => point.TryGetLocalPointIn(Vector3d.Zero, invalid, out _));
    }

    [Fact]
    public void InverseBoundsAndSupport_ShouldUseTheSameExactGeometry()
    {
        Vector3d reference = Raw(3865470566, -Q / 3, Q / 7), offset = Raw(-Q / 5, Q / 3, Q / 11);
        Vector3d owner = Raw(2863311531, 2576980378, Q / 2), part = Raw(Q / 3, 3 * Q, 2 * Q);
        foreach (FixedQuaternion placement in new[] { FixedQuaternion.Identity, Rotation })
        foreach (FixedQuaternion geometry in new[] { FixedQuaternion.Identity, OtherRotation })
        foreach (bool negative in new[] { false, true })
        {
            Vector3d actualOwner = owner;
            if (negative) actualOwner.X = -actualOwner.X;
            Vector3d translation = Raw(Q / 9, -2 * Q, 3 * Q);
            var frame = new ScaledRigidFrame3d(translation, placement, offset, actualOwner, geometry, reference, part);
            (BigInteger[] center, BigInteger d) = OraclePoint(reference, reference, offset, translation,
                placement, geometry, actualOwner, part);
            (BigInteger[,] matrix, BigInteger rotationD) = Basis(geometry);
            (BigInteger[], BigInteger[]) SourceOracle(Vector3d world)
            {
                BigInteger[] n = new BigInteger[3], denominators = new BigInteger[3];
                for (int axis = 0; axis < 3; axis++)
                {
                    BigInteger scale = (BigInteger)actualOwner[axis].m_rawValue * part[axis].m_rawValue;
                    denominators[axis] = d * rotationD * scale;
                    n[axis] = reference[axis].m_rawValue * denominators[axis];
                    for (int worldAxis = 0; worldAxis < 3; worldAxis++)
                        n[axis] += Q2 * matrix[worldAxis, axis] * (world[worldAxis].m_rawValue * d - center[worldAxis]);
                }
                return (n, denominators);
            }
            Vector3d worldPoint = Raw(2 * Q, -Q / 3, Q / 5);
            (BigInteger[] expected, BigInteger[] denominators) = SourceOracle(worldPoint);
            frame.GetSourcePointNumerators(worldPoint, out Signed576 sx, out Signed576 sy, out Signed576 sz,
                out Signed576 dx, out Signed576 dy, out Signed576 dz);
            Assert.Equal(expected, new[] { Big(sx), Big(sy), Big(sz) });
            Assert.Equal(denominators, new[] { Big(dx), Big(dy), Big(dz) });
            for (int axis = 0; axis < 3; axis++)
            {
                frame.GetSourceCoordinateTransform(axis, out Signed576 cx, out Signed576 cy, out Signed576 cz,
                    out Signed576 co, out Signed576 cd);
                BigInteger transformed = Big(cx) * worldPoint.X.m_rawValue + Big(cy) * worldPoint.Y.m_rawValue
                    + Big(cz) * worldPoint.Z.m_rawValue + Big(co);
                Assert.Equal(expected[axis] / Q2, transformed);
                Assert.Equal(denominators[axis] / Q2, Big(cd));
            }

            Vector3d local = Raw(Q / 2, -Q, Q / 3);
            RationalPoint3d localWorld = frame.GetScaledLocalPoint(local);
            BigInteger[] worldExpected = (BigInteger[])center.Clone();
            for (int axis = 0; axis < 3; axis++)
                for (int localAxis = 0; localAxis < 3; localAxis++)
                    worldExpected[axis] += matrix[axis, localAxis] * local[localAxis].m_rawValue * (d / rotationD);
            Assert.Equal(worldExpected, Coordinates(localWorld));
            Assert.Equal(d, Big(localWorld.Denominator));
            Assert.True(frame.TryGetSourceBoundsFromScaledLocal(local, local, out FixedBoundBox localSourceBounds));
            Assert.True(frame.TryGetScaledLocalPointIn(worldPoint, out Vector3d scaledPoint));
            for (int axis = 0; axis < 3; axis++)
            {
                BigInteger scale = (BigInteger)actualOwner[axis].m_rawValue * part[axis].m_rawValue;
                BigInteger sourceN = reference[axis].m_rawValue * scale + local[axis].m_rawValue * Q2;
                BigInteger sourceD = scale;
                if (sourceD.Sign < 0) { sourceN = -sourceN; sourceD = -sourceD; }
                Assert.Equal((long)Floor(sourceN, sourceD), localSourceBounds.Min[axis].m_rawValue);
                Assert.Equal((long)-Floor(-sourceN, sourceD), localSourceBounds.Max[axis].m_rawValue);
                BigInteger localN = 0;
                for (int worldAxis = 0; worldAxis < 3; worldAxis++)
                    localN += matrix[worldAxis, axis] * (worldPoint[worldAxis].m_rawValue * d - center[worldAxis]);
                Assert.Equal((long)Round(localN, d * rotationD), scaledPoint[axis].m_rawValue);
            }

            Vector3d direction = Raw(3 * Q, -2 * Q, Q);
            WideAxis3 projected = frame.GetSourceDirectionNumerators(direction);
            Signed320[] coefficients = { projected.X, projected.Y, projected.Z };
            for (int axis = 0; axis < 3; axis++)
            {
                BigInteger coefficient = 0;
                for (int worldAxis = 0; worldAxis < 3; worldAxis++)
                    coefficient += matrix[worldAxis, axis] * direction[worldAxis].m_rawValue;
                coefficient *= (BigInteger)actualOwner[axis].m_rawValue * part[axis].m_rawValue;
                Assert.Equal(coefficient, Big(coefficients[axis]));
            }

            Vector3d min = Raw(-Q, -2 * Q, -Q / 2), max = Raw(2 * Q, Q, Q / 3);
            FixedBoundBox bounds = frame.GetWorldBounds(min, max);
            AssertBounds(bounds, min, max, source =>
            {
                (BigInteger[] n, BigInteger denominator) = OraclePoint(source, reference, offset, translation,
                    placement, geometry, actualOwner, part);
                return (n, new[] { denominator, denominator, denominator });
            });
            Assert.True(frame.TryGetSourceBounds(min, max, out FixedBoundBox sourceBounds));
            AssertBounds(sourceBounds, min, max, SourceOracle);
        }
    }

    [Fact]
    public void CacheEquivalenceAndMappings_ShouldUseExactCoordinatesRatherThanRoundedAgreement()
    {
        Vector3d origin = Raw(Q, -Q, 2 * Q), reference = Raw(3865470566, Q / 3, -Q / 7);
        ScaledRigidFrame3d rigid = ScaledRigidFrame3d.Rigid(origin, Rotation);
        var centered = new ScaledRigidFrame3d(origin, Rotation, reference, Vector3d.One, Rotation, reference, Vector3d.One);
        Assert.True(rigid.IsRigid); Assert.True(centered.IsRigid);
        Assert.False(rigid.HasLocalPlacement); Assert.True(centered.HasLocalPlacement);
        Assert.Equal(origin, rigid.RigidOrigin); Assert.Equal(Rotation, rigid.Rotation);
        Assert.True(rigid.HasSameMapping(centered)); Assert.True(centered.HasSameMapping(rigid));
        var opposite = new FixedQuaternion(-Rotation.X, -Rotation.Y, -Rotation.Z, -Rotation.W);
        Assert.True(rigid.HasSameMapping(ScaledRigidFrame3d.Rigid(origin, opposite)));
        RationalPoint3d scalar = RationalPoint3d.FromScalarPoint(origin);
        Assert.True(scalar.TryGetPoint(out Vector3d restoredScalar));
        Assert.Equal(origin, restoredScalar);
        for (int axis = 0; axis < 3; axis++)
        {
            Vector3d differentOrigin = origin; differentOrigin[axis] += Fixed64.FromRaw(1);
            Assert.False(rigid.HasSameMapping(ScaledRigidFrame3d.Rigid(differentOrigin, Rotation)));
            var unit = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
            Vector3d scale = Vector3d.One; scale[axis] = (Fixed64)2;
            var changedScale = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
                scale, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
            Assert.False(changedScale.IsRigid); Assert.False(unit.HasSameMapping(changedScale));
            Vector3d offset = Vector3d.Zero; offset[axis] = Fixed64.FromRaw(1);
            var shifted = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, offset,
                Vector3d.One, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
            Assert.False(shifted.IsRigid);
        }
        var separatelyRotated = new ScaledRigidFrame3d(origin, Rotation, reference, Vector3d.One,
            OtherRotation, reference, Vector3d.One);
        Assert.False(separatelyRotated.IsRigid);
        var fractional = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Raw(Q / 2, Q / 2, Q / 2), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Assert.True(fractional.IsScaledLocalPointExact(Raw(2, 4, 6), Raw(1, 2, 3)));
        for (int axis = 0; axis < 3; axis++)
        {
            Vector3d input = Raw(2, 4, 6); input[axis] += Fixed64.FromRaw(1);
            Assert.False(fractional.IsScaledLocalPointExact(input, Raw(1, 2, 3)));
        }
    }

    [Fact]
    public void Bounds_ShouldClipAtBothDomainFacesAndRejectSingularInverses()
    {
        foreach (long originRaw in new[] { long.MinValue, long.MaxValue })
        {
            Vector3d origin = Raw(originRaw, originRaw, originRaw);
            ScaledRigidFrame3d frame = ScaledRigidFrame3d.Rigid(origin, FixedQuaternion.Identity);
            FixedBoundBox world = frame.GetWorldBounds(Raw(-Q, -Q, -Q), Raw(Q, Q, Q));
            Assert.Equal(originRaw == long.MinValue ? long.MinValue : long.MaxValue - Q, world.Min.X.m_rawValue);
            Assert.Equal(originRaw == long.MinValue ? long.MinValue + Q : long.MaxValue, world.Max.X.m_rawValue);
            Assert.True(frame.TryGetSourceBounds(-origin, -origin, out _));
        }
        var fractional = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Raw(Q / 2, Q / 2, Q / 2), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        FixedBoundBox half = fractional.GetWorldBounds(Raw(1, 1, 1), Raw(3, 3, 3));
        Assert.Equal(Raw(0, 0, 0), half.Min); Assert.Equal(Raw(2, 2, 2), half.Max);
        for (int axis = 0; axis < 3; axis++)
        {
            Vector3d scale = Vector3d.One; scale[axis] = Fixed64.Zero;
            var singular = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
                scale, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
            Assert.False(singular.TryGetSourceBoundsFromScaledLocal(Vector3d.Zero, Vector3d.One, out _));
            Assert.False(singular.TryGetSourceBounds(Vector3d.Zero, Vector3d.One, out _));
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RigidWorldBounds_ShouldMatchExactOutwardProjectionAtEveryDomainFace(bool maximum)
    {
        long face = maximum ? long.MaxValue : long.MinValue;
        Vector3d origin = Raw(face, face, face), min = Raw(-Q - 1, -2 * Q, -3 * Q + 1), max = Raw(4 * Q + 1, Q, 2 * Q);
        foreach (FixedQuaternion rotation in new[] { FixedQuaternion.Identity, Rotation, OtherRotation })
        {
            var frame = ScaledRigidFrame3d.Rigid(origin, rotation);
            FixedBoundBox bounds = frame.GetWorldBounds(min, max);
            var (basis, denominator) = Basis(rotation);
            AssertBounds(bounds, min, max, point =>
            {
                BigInteger[] n = new BigInteger[3], d = { denominator, denominator, denominator };
                for (int axis = 0; axis < 3; axis++)
                    n[axis] = origin[axis].m_rawValue * denominator
                        + basis[axis, 0] * point.X.m_rawValue + basis[axis, 1] * point.Y.m_rawValue + basis[axis, 2] * point.Z.m_rawValue;
                return (n, d);
            });
        }
    }

    private static void AssertBounds(FixedBoundBox actual, Vector3d min, Vector3d max,
        Func<Vector3d, (BigInteger[] Numerators, BigInteger[] Denominators)> oracle)
    {
        BigInteger[] expectedMin = { long.MaxValue, long.MaxValue, long.MaxValue };
        BigInteger[] expectedMax = { long.MinValue, long.MinValue, long.MinValue };
        for (int corner = 0; corner < 8; corner++)
        {
            Vector3d point = new((corner & 1) == 0 ? min.X : max.X, (corner & 2) == 0 ? min.Y : max.Y,
                (corner & 4) == 0 ? min.Z : max.Z);
            (BigInteger[] n, BigInteger[] d) = oracle(point);
            for (int axis = 0; axis < 3; axis++)
            {
                if (d[axis].Sign < 0) { n[axis] = -n[axis]; d[axis] = -d[axis]; }
                BigInteger q = BigInteger.DivRem(n[axis], d[axis], out BigInteger r);
                BigInteger floor = q - (r.Sign < 0 ? 1 : 0), ceil = q + (r.Sign > 0 ? 1 : 0);
                floor = BigInteger.Max(long.MinValue, BigInteger.Min(long.MaxValue, floor));
                ceil = BigInteger.Max(long.MinValue, BigInteger.Min(long.MaxValue, ceil));
                expectedMin[axis] = BigInteger.Min(expectedMin[axis], floor);
                expectedMax[axis] = BigInteger.Max(expectedMax[axis], ceil);
            }
        }
        for (int axis = 0; axis < 3; axis++)
        {
            Assert.Equal((long)expectedMin[axis], actual.Min[axis].m_rawValue);
            Assert.Equal((long)expectedMax[axis], actual.Max[axis].m_rawValue);
        }
    }


    private static RationalPoint3d Check(Vector3d source, Vector3d reference, Vector3d offset, Vector3d translation,
        FixedQuaternion placementRotation, FixedQuaternion geometryRotation, Vector3d owner, Vector3d part)
    {
        var frame = new ScaledRigidFrame3d(translation, placementRotation, offset, owner, geometryRotation, reference, part);
        RationalPoint3d actual = frame.GetPoint(source);
        (BigInteger[,] g, BigInteger gd) = Basis(geometryRotation);
        (BigInteger[] expected, BigInteger denominator) = OraclePoint(source, reference, offset, translation,
            placementRotation, geometryRotation, owner, part);
        Assert.Equal(denominator, Big(actual.Denominator));
        Assert.Equal(denominator, Big(frame.Denominator));
        Assert.Equal(denominator / Q2, Big(frame.RotationDenominator));
        Assert.Equal(geometryRotation, frame.GeometryRotation);
        Assert.Equal(gd, Big(frame.GeometryBasis.Denominator));
        Assert.Equal(expected, Coordinates(actual));
        frame.GetScaledLocalPointNumerators(source, out Signed192 lx, out Signed192 ly, out Signed192 lz);
        Signed192[] localNumerators = { lx, ly, lz };
        for (int axis = 0; axis < 3; axis++)
            Assert.Equal(((BigInteger)source[axis].m_rawValue - reference[axis].m_rawValue)
                * owner[axis].m_rawValue * part[axis].m_rawValue, Big(localNumerators[axis]));
        CheckMaterialization(expected, denominator, actual.TryGetPoint(out Vector3d value), value);
        Assert.Equal(actual.TryGetPoint(out Vector3d direct), frame.TryGetPoint(source, out Vector3d framed));
        Assert.Equal(direct, framed);
        foreach (FixedQuaternion target in new[] { FixedQuaternion.Identity, OtherRotation })
        {
            Vector3d targetOrigin = Raw(long.MinValue + Q, -Q, long.MaxValue - Q);
            actual.GetLocalPointNumerators(targetOrigin, target, out Signed576 x, out Signed576 y, out Signed576 z, out Signed320 targetDenominator);
            (BigInteger[,] r, BigInteger rd) = Basis(target);
            BigInteger[] targetExpected = new BigInteger[3];
            for (int axis = 0; axis < 3; axis++)
                for (int world = 0; world < 3; world++)
                    targetExpected[axis] += r[world, axis] * (expected[world] - targetOrigin[world].m_rawValue * denominator);
            Assert.Equal(denominator * rd, Big(targetDenominator));
            Assert.Equal(targetExpected, new[] { Big(x), Big(y), Big(z) });
            CheckMaterialization(targetExpected, denominator * rd,
                actual.TryGetLocalPointIn(targetOrigin, target, out Vector3d localValue), localValue);
        }
        return actual;
    }

    private static (BigInteger[], BigInteger) OraclePoint(Vector3d source, Vector3d reference, Vector3d offset,
        Vector3d translation, FixedQuaternion placementRotation, FixedQuaternion geometryRotation, Vector3d owner, Vector3d part)
    {
        (BigInteger[,] p, BigInteger pd) = Basis(placementRotation);
        (BigInteger[,] g, BigInteger gd) = Basis(geometryRotation);
        bool same = placementRotation == geometryRotation;
        BigInteger denominator = Q2 * gd * (same ? BigInteger.One : pd);
        BigInteger[] expected = new BigInteger[3];
        for (int axis = 0; axis < 3; axis++)
        {
            expected[axis] = translation[axis].m_rawValue * denominator;
            for (int local = 0; local < 3; local++)
            {
                BigInteger displacement = ((BigInteger)source[local].m_rawValue - reference[local].m_rawValue)
                    * owner[local].m_rawValue * part[local].m_rawValue;
                BigInteger placement = (BigInteger)offset[local].m_rawValue * owner[local].m_rawValue * Q;
                expected[axis] += same ? g[axis, local] * (displacement + placement)
                    : g[axis, local] * displacement * pd + p[axis, local] * placement * gd;
            }
        }
        return (expected, denominator);
    }

    [Fact]
    public void DirectionAndFrameBounds_ShouldRemainExactBeyondWorldFaces()
    {
        Vector3d reference = Raw(Q, -Q, Q / 3), owner = Raw(2863311531, 2576980378, -Q / 2);
        Vector3d part = Raw(Q / 3, 2 * Q, Q);
        foreach (long face in new[] { long.MinValue, long.MaxValue })
        {
            var frame = new ScaledRigidFrame3d(Raw(face, face, face), Rotation, reference, owner,
                OtherRotation, reference, part);
            Vector3d min = Raw(-2 * Q, -Q, 0), max = Raw(3 * Q, 2 * Q, Q);
            Assert.True(frame.TryGetBoundsIn(min, max, frame, out FixedBoundBox bounds));
            Assert.Equal(min, bounds.Min); Assert.Equal(max, bounds.Max);
            WideAxis3 authoredRay = new(Signed320.ExtendValue(Signed192.Signed(3 * Q)),
                Signed320.ExtendValue(Signed192.Signed(-2 * Q)), Signed320.ExtendValue(Signed192.Signed(1)));
            WideAxis3 transformed = frame.GetWorldDirectionNumerators(authoredRay);
            var (matrix, _) = Basis(OtherRotation);
            BigInteger[] raw = { 3 * Q, -2 * Q, 1 };
            for (int axis = 0; axis < 3; axis++)
            {
                BigInteger expected = 0;
                for (int column = 0; column < 3; column++)
                    expected += matrix[axis, column] * raw[column] * owner[column].m_rawValue * part[column].m_rawValue;
                Assert.Equal(expected, Big(axis == 0 ? transformed.X : axis == 1 ? transformed.Y : transformed.Z));
            }
        }
        Vector3d zero = Vector3d.One; zero.Z = Fixed64.Zero;
        var singular = new ScaledRigidFrame3d(Vector3d.Zero, Rotation, Vector3d.Zero, zero,
            Rotation, Vector3d.Zero, Vector3d.One);
        Assert.False(ScaledRigidFrame3d.Rigid(Vector3d.Zero, Rotation).TryGetBoundsIn(Vector3d.Zero,
            Vector3d.One, singular, out _));
    }
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ExactLatticeConversion_ShouldRejectTheFirstFractionalCoordinateOnEveryAxis(int axis)
    {
        Vector3d scale = Vector3d.One; scale[axis] = Fixed64.Half;
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, scale,
            FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Vector3d authored = Vector3d.One; authored[axis] = Fixed64.FromRaw(1);
        RationalPoint3d point = frame.GetPoint(authored);
        Assert.False(point.TryGetExactLocalPointIn(Vector3d.Zero, FixedQuaternion.Identity, out _));
        authored[axis] = Fixed64.FromRaw(2);
        Assert.True(frame.GetPoint(authored).TryGetExactLocalPointIn(Vector3d.Zero, FixedQuaternion.Identity, out Vector3d exact));
        Assert.Equal(Fixed64.FromRaw(1), exact[axis]);
    }

    [Fact]
    public void RigidWorldDirection_ShouldPreserveThePrimitiveQuaternionBasisWithoutScaleFactors()
    {
        var frame = ScaledRigidFrame3d.Rigid(Raw(long.MaxValue, long.MinValue, Q), Rotation);
        WideAxis3 ray = new(Signed320.ExtendValue(Signed192.Signed(3)), Signed320.ExtendValue(Signed192.Signed(-2)),
            Signed320.ExtendValue(Signed192.Signed(5)));
        WideAxis3 actual = frame.GetWorldDirectionNumerators(ray);
        var (matrix, _) = Basis(Rotation);
        for (int axis = 0; axis < 3; axis++)
            Assert.Equal(3 * matrix[axis, 0] - 2 * matrix[axis, 1] + 5 * matrix[axis, 2],
                Big(axis == 0 ? actual.X : axis == 1 ? actual.Y : actual.Z));
    }

    private static void CheckMaterialization(BigInteger[] numerators, BigInteger denominator, bool success, Vector3d value)
    {
        BigInteger[] rounded = numerators.Select(n => Round(n, denominator)).ToArray();
        bool expected = rounded.All(v => v >= long.MinValue && v <= long.MaxValue);
        Assert.Equal(expected, success);
        if (expected)
            for (int axis = 0; axis < 3; axis++) Assert.Equal((long)rounded[axis], value[axis].m_rawValue);
        else
            Assert.Equal(Vector3d.Zero, value);
    }

    private static (BigInteger[,], BigInteger) Basis(FixedQuaternion q)
    {
        if (q == FixedQuaternion.Identity)
            return (new BigInteger[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } }, BigInteger.One);
        BigInteger x = q.X.m_rawValue, y = q.Y.m_rawValue, z = q.Z.m_rawValue, w = q.W.m_rawValue;
        return (new[,] { { x*x-y*y-z*z+w*w, 2*(x*y-z*w), 2*(x*z+y*w) },
            { 2*(x*y+z*w), y*y-x*x-z*z+w*w, 2*(y*z-x*w) },
            { 2*(x*z-y*w), 2*(y*z+x*w), z*z-x*x-y*y+w*w } }, x*x+y*y+z*z+w*w);
    }

    private static Vector3d Raw(long x, long y, long z) => new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));
    private static BigInteger[] Coordinates(RationalPoint3d point) => new[] { Big(point.X), Big(point.Y), Big(point.Z) };
    private static BigInteger[] Subtract(BigInteger[] a, BigInteger[] b) => a.Zip(b, (x, y) => x - y).ToArray();
    private static BigInteger[] Cross(BigInteger[] a, BigInteger[] b) => new[] { a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0] };
    private static BigInteger Dot(BigInteger[] a, BigInteger[] b) => a.Zip(b, (x, y) => x*y).Aggregate(BigInteger.Zero, (x, y) => x+y);
    private static BigInteger Floor(BigInteger n, BigInteger d)
    {
        BigInteger q = BigInteger.DivRem(n, d, out BigInteger r);
        return r.Sign < 0 ? q - 1 : q;
    }
    private static BigInteger Round(BigInteger n, BigInteger d)
    {
        if (d.Sign < 0) { n = -n; d = -d; }
        BigInteger q = BigInteger.DivRem(BigInteger.Abs(n), d, out BigInteger r);
        if (2*r > d || (2*r == d && !q.IsEven)) q++;
        return n.Sign < 0 ? -q : q;
    }
    private static BigInteger Big(Signed192 v) => Words(v.Low, v.Middle, v.High);
    private static BigInteger Big(Signed320 v) => Words(v.Word0, v.Word1, v.Word2, v.Word3, v.Word4);
    private static BigInteger Big(Signed576 v) => Words(v.Word0, v.Word1, v.Word2, v.Word3, v.Word4, v.Word5, v.Word6, v.Word7, v.Word8);
    private static BigInteger Words(params ulong[] words)
    {
        BigInteger value = 0;
        for (int i = words.Length - 1; i >= 0; i--) value = (value << 64) + words[i];
        return (words[^1] & (1UL << 63)) == 0 ? value : value - (BigInteger.One << (64*words.Length));
    }
}

