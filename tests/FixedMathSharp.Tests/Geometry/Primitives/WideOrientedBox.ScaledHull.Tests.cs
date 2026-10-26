using FixedMathSharp.Geometry;
using Xunit;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class WideOrientedBoxScaledHullTests
{
    private static readonly Vector3d[] Cube = { new(-1, -1, -1), new(1, -1, -1), new(1, 1, -1), new(-1, 1, -1),
        new(-1, -1, 1), new(1, -1, 1), new(1, 1, 1), new(-1, 1, 1) };
    private static readonly int[] Faces = { 0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
        3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5 };
    private static readonly int[] Edges = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4,
        0, 4, 1, 5, 2, 6, 3, 7 };

    [Theory]
    [InlineData(0)]
    [InlineData(17)]
    [InlineData(-43)]
    [InlineData(90)]
    public void NativeYawFrame_ShouldRetainTheExactScalarTransformAndPhysicalPlanes(int degrees)
    {
        Fixed64 yaw = Fixed64.FromFraction(degrees, 180) * Fixed64.Pi;
        Vector3d origin = new((Fixed64)2000000000, -Fixed64.Quarter, (Fixed64)(-1900000000));
        var frame = new NativePrismYawFrame3d(origin, yaw);
        Assert.True(frame.RotationDenominator.Equals(Signed192.Signed(1)));
        BigInteger q = BigInteger.One << 32;
        BigInteger cosine = FixedMath.Cos(yaw).m_rawValue, sine = FixedMath.Sin(yaw).m_rawValue;
        foreach (Vector3d local in new[] { Vector3d.Zero, new Vector3d(Fixed64.FromRaw(long.MinValue + 7),
            Fixed64.FromRaw(3), Fixed64.FromRaw(long.MaxValue - 11)), new Vector3d(2, -3, 5) })
        {
            RationalPoint3d point = frame.GetPoint(local);
            BigInteger denominator = Integer(point.Denominator);
            Assert.Equal(q * q, denominator);
            Assert.Equal(((BigInteger)origin.X.m_rawValue * q + cosine * local.X.m_rawValue - sine * local.Z.m_rawValue) * denominator,
                Integer(point.X) * q);
            Assert.Equal(((BigInteger)origin.Z.m_rawValue * q + sine * local.X.m_rawValue + cosine * local.Z.m_rawValue) * denominator,
                Integer(point.Z) * q);
            Assert.Equal(((BigInteger)origin.Y.m_rawValue + local.Y.m_rawValue) * denominator, Integer(point.Y));
        }
        var triangle = new FixedTriangle(new(-1, -2, 3), new(2, 1, 4), new(0, 3, -2));
        WideDirection3d normal = frame.GetFaceNormal(triangle);
        Assert.True(WideDirection3d.Dot(normal, frame.GetEdgeDirection(triangle.A, triangle.B)).IsZero);
        Assert.True(WideDirection3d.Dot(normal, frame.GetEdgeDirection(triangle.A, triangle.C)).IsZero);
        Span<Signed832> values = stackalloc Signed832[3];
        var face = new FixedTriangle(Vector3d.Zero, new(2, 0, 0), new(0, 2, 0));
        Assert.True(frame.TryGetTriangleHalfPlanes(face, frame.GetPoint(new(Fixed64.Half, Fixed64.Half, Fixed64.Zero)), values));
        foreach (Signed832 value in values) Assert.True(value.Sign > 0);
        Assert.True(frame.TryGetTriangleHalfPlanes(face, frame.GetPoint(new(2, 2, 0)), values));
        Assert.Contains(values.ToArray(), value => value.Sign < 0);
        Assert.False(frame.TryGetTriangleHalfPlanes(default, frame.GetPoint(Vector3d.Zero), values));

        static BigInteger Integer(Signed320 value)
        {
            BigInteger result = (BigInteger)value.Word4 << 256 | (BigInteger)value.Word3 << 192
                | (BigInteger)value.Word2 << 128 | (BigInteger)value.Word1 << 64 | value.Word0;
            return value.Sign < 0 ? result - (BigInteger.One << 320) : result;
        }
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    [InlineData(3, false)]
    [InlineData(3, true)]
    [InlineData(4, false)]
    [InlineData(4, true)]
    [InlineData(5, false)]
    [InlineData(5, true)]
    [InlineData(6, false)]
    [InlineData(6, true)]
    [InlineData(7, false)]
    [InlineData(7, true)]
    [InlineData(8, false)]
    [InlineData(8, true)]
    [InlineData(9, false)]
    [InlineData(9, true)]
    public void RetainedRectangleWindows_ShouldPreserveExactCornersAcrossEquivalentAxesAndReversedEdges(int window, bool negateAxis)
    {
        var source = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        Vector3d origin = Vector3d.Backward;
        var target = new NativePrismYawFrame3d(origin, Fixed64.Zero);
        Vector3d[] prism = Array.ConvertAll(Cube, p => new Vector3d(p.X * 4, p.Y * 2, p.Z));
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(source, Cube, Faces, Edges,
            target, prism, Faces, Edges, out var selected));
        if (negateAxis) selected = new(-selected.Axis, !selected.Negate, selected.Overlap,
            selected.SquaredAxisLength, selected.CommonDenominator);
        Fixed64 left = window == 6 ? -Fixed64.Two : window == 9 ? -Fixed64.One / 8 : -Fixed64.Half;
        Fixed64 right = window == 6 || window is 7 or 8 ? Fixed64.Two : -left;
        var rectangle = new FixedTriangle(new(left, -Fixed64.Quarter, Fixed64.One),
            new(right, -Fixed64.Quarter, Fixed64.One), new(left, Fixed64.Quarter, Fixed64.One));
        RationalSegmentInterval clip = window switch
        {
            1 => Range(0, 1, 1, 2), 2 => Range(1, 2, 1, 1), 3 => Range(1, 4, 3, 4),
            4 => Range(0, 1, 0, 1), 5 => Range(1, 1, 1, 1), 8 => Range(1, 2, 1, 1), _ => default
        };
        Fixed64 minimum = window switch { 2 => Fixed64.Zero, 3 => -Fixed64.Quarter,
            5 => Fixed64.Half, 6 => -Fixed64.One, 8 => 3 * Fixed64.Quarter, _ => left };
        Fixed64 maximum = window switch { 1 => Fixed64.Zero, 3 => Fixed64.Quarter,
            4 => -Fixed64.Half, 6 or 7 or 8 => Fixed64.One, _ => right };
        HashSet<(Vector3d Moving, Vector3d Wall)>? expected = null;
        foreach (bool reverse in new[] { false, true })
        {
            var edges = (int[])Edges.Clone();
            if (reverse)
                for (int edge = 0; edge < edges.Length; edge += 2) (edges[edge], edges[edge + 1]) = (edges[edge + 1], edges[edge]);
            var rows = new List<FixedContactAnchors>();
            Assert.Equal(1, WideOrientedBox.VisitProjectedHullRectangleContacts(source, Vector3d.Zero, Cube, Faces, edges,
                target, origin, rectangle, clip, selected, rows, static (pool, row) => pool.Add(row), admissionOnly: true));
            Assert.Empty(rows);
            Assert.True(WideOrientedBox.VisitProjectedHullRectangleContacts(source, Vector3d.Zero, Cube, Faces, edges,
                target, origin, rectangle, clip, selected, rows, static (pool, row) => pool.Add(row)) > 0);
            var corners = new HashSet<(Vector3d Moving, Vector3d Wall)>();
            foreach (FixedContactAnchors row in rows)
            {
                Assert.Equal(Vector3d.Backward, row.Normal); Assert.Equal(Fixed64.One, row.Depth);
                Assert.True(row.FirstAnchor.TryGetPoint(out Vector3d moving));
                Assert.True(row.SecondAnchor.TryGetPoint(out Vector3d wall));
                Assert.Equal(new Vector3d(moving.X, moving.Y, Fixed64.Zero), wall);
                Assert.Equal(-Fixed64.One, moving.Z);
                Assert.True(wall.X == minimum || wall.X == maximum);
                Assert.Equal(Fixed64.Quarter, wall.Y.Abs());
                corners.Add((moving, wall));
            }
            Assert.Equal(minimum == maximum ? 2 : 4, corners.Count);
            if (expected != null) Assert.True(expected.SetEquals(corners));
            expected = corners;
        }

        static RationalSegmentInterval Range(long a, long b, long c, long d) => new(
            Signed832.ExtendValue(Signed192.Signed(a)), Signed832.ExtendValue(Signed192.Signed(b)),
            Signed832.ExtendValue(Signed192.Signed(c)), Signed832.ExtendValue(Signed192.Signed(d)));
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void RectangleSampler_ShouldKeepAdmissionSeparateFromUnrepresentableSourceOrTargetCharts(int window, bool badTarget)
    {
        Vector3d sourceOrigin = Vector3d.Right * Fixed64.Two, targetOrigin = sourceOrigin + Vector3d.Backward;
        var source = ScaledRigidFrame3d.Rigid(sourceOrigin, FixedQuaternion.Identity);
        var target = new NativePrismYawFrame3d(targetOrigin, Fixed64.Zero);
        Vector3d[] prism = Array.ConvertAll(Cube, p => new Vector3d(p.X * 4, p.Y * 2, p.Z));
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(source, Cube, Faces, Edges,
            target, prism, Faces, Edges, out var selected));
        Fixed64 width = window == 2 ? Fixed64.Half : (Fixed64)4;
        Fixed64 height = window == 0 ? Fixed64.Two : Fixed64.Quarter;
        var rectangle = new FixedTriangle(new(-width, -height, Fixed64.One),
            new(width, -height, Fixed64.One), new(-width, height, Fixed64.One));
        // Every retained witness has positive world X; a chart at MinValue
        // cannot represent its local displacement, although the geometry intersects.
        Vector3d outside = new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);
        var rows = new List<FixedContactAnchors>();
        Assert.True(WideOrientedBox.VisitProjectedHullRectangleContacts(source, sourceOrigin, Cube, Faces, Edges,
            target, targetOrigin, rectangle, default, selected, rows, static (pool, row) => pool.Add(row)) > 0);
        foreach (FixedContactAnchors row in rows)
        { Assert.True(row.FirstAnchor.TryGetPoint(out _)); Assert.True(row.SecondAnchor.TryGetPoint(out _)); }
        rows.Clear();
        Assert.Equal(1, WideOrientedBox.VisitProjectedHullRectangleContacts(source, badTarget ? sourceOrigin : outside,
            Cube, Faces, Edges, target, badTarget ? outside : targetOrigin, rectangle, default, selected,
            rows, static (pool, row) => pool.Add(row), admissionOnly: true));
        Assert.Equal(0, WideOrientedBox.VisitProjectedHullRectangleContacts(source, badTarget ? sourceOrigin : outside,
            Cube, Faces, Edges, target, badTarget ? outside : targetOrigin, rectangle, default, selected,
            rows, static (pool, row) => pool.Add(row)));
        Assert.Empty(rows);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void NativeYawIdentityHull_ShouldMatchRigidOverlapDepthAndFiniteWitnesses(int separation)
    {
        var first = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        Vector3d offset = Vector3d.Right * (Fixed64)separation;
        var second = new NativePrismYawFrame3d(offset, Fixed64.Zero);
        bool expected = separation <= 2;
        Assert.Equal(expected, WideOrientedBox.TryGetConvexHullPenetration(first, Cube, Faces, Edges,
            second, Cube, Faces, Edges, out var selected, true));
        Assert.Equal(expected, selected.HasValue);
        if (!expected) return;
        Assert.Equal((Fixed64)(2 - separation), selected.GetRoundedDepth(out bool clamped));
        Assert.False(clamped);
        var contacts = new List<FixedContactAnchors>();
        Assert.True(WideOrientedBox.VisitConvexHullContacts(first, Vector3d.Zero, Cube, Faces, Edges,
            second, offset, Cube, Faces, Edges, selected, contacts, static (pool, contact) => pool.Add(contact)) > 0);
        foreach (FixedContactAnchors contact in contacts)
        {
            Assert.Equal((Fixed64)(2 - separation), contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out var a));
            Assert.True(contact.SecondAnchor.TryGetPoint(out var b));
            Assert.InRange(a.X.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            Assert.InRange((b.X - offset.X).m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
        }
        Vector3d[] quad = { new(-1, 0, -1), new(1, 0, -1), new(1, 0, 1), new(-1, 0, 1) };
        int[] triangles = { 0, 1, 2, 0, 2, 3 }, edges = { 0, 1, 1, 2, 2, 3, 3, 0 };
        Assert.Equal(expected, WideOrientedBox.TryGetConvexHullPenetration(second, quad, triangles, edges,
            first, quad, triangles, edges, out _, true, planarPair: true));
    }

    [Fact]
    public void WidenedRigidPenetration_ShouldPreserveBothSignedExitsAndUnsetState()
    {
        var unset = new WideScaledHullPenetration(default(WidePointSpanPenetration));
        Assert.False(unset.HasValue);
        foreach (int side in new[] { -1, 1 })
        {
            Assert.True(WideOrientedBox.TryGetConvexHullPenetration(Vector3d.Zero, FixedQuaternion.Identity,
                Cube, Faces, Edges, Vector3d.Right * (Fixed64)side, FixedQuaternion.Identity,
                Cube, Faces, Edges, out var rigid));
            var wide = new WideScaledHullPenetration(rigid);
            Assert.True(wide.HasValue); Assert.Equal(rigid.Negate, wide.Negate);
            Assert.Equal((Fixed64)side * Vector3d.Right, wide.GetNormal());
            Assert.True(wide.Overlap.Equals(Signed832.ExtendValue(rigid.ExactOverlap)));
            Assert.True(wide.SquaredAxisLength.Equals(Signed832.ExtendValue(rigid.ExactSquaredAxisLength)));
            Assert.True(wide.CommonDenominator.Equals(Signed576.ExtendValue(rigid.ExactCommonDenominator)));
            Assert.Equal(rigid.GetRoundedDepth(out bool rigidClamped), wide.GetRoundedDepth(out bool wideClamped));
            Assert.Equal(rigidClamped, wideClamped);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void RetainedRectangleSampler_ShouldKeepOnlyFiniteSupportAndHandleAnEnclosedPatch(bool native, bool affine)
    {
        Vector3d rectangleOrigin = Vector3d.Backward;
        Vector3d[] prism = new Vector3d[Cube.Length];
        for (int i = 0; i < Cube.Length; i++) prism[i] = new(Cube[i].X * 4, Cube[i].Y * 2, Cube[i].Z);
        for (int domain = 0; domain < 6; domain++)
        {
            Fixed64 extent = affine ? Fixed64.Half : Fixed64.One;
            Vector3d origin = new(domain == 0 ? -Fixed64.Two : Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
            var source = new ScaledRigidFrame3d(origin, FixedQuaternion.Identity, Vector3d.Zero,
                Vector3d.One * extent, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
            Fixed64 height = domain == 2 ? Fixed64.Quarter : domain == 4 ? Fixed64.Zero : Fixed64.Two;
            FixedTriangle rectangle = domain == 5
                ? new(new Vector3d(0, -2, -1), new(0, -2, 1), new(0, 2, -1))
                : new(new Vector3d((Fixed64)(-4), -height, Fixed64.One),
                    new((Fixed64)4, -height, Fixed64.One), new((Fixed64)(-4), height, Fixed64.One));
            Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
            RationalSegmentInterval clip = domain == 3 ? default : domain == 2
                ? new(Signed832.ExtendValue(Signed192.Signed(15)), Signed832.ExtendValue(Signed192.Signed(32)),
                    Signed832.ExtendValue(Signed192.Signed(17)), Signed832.ExtendValue(Signed192.Signed(32)))
                : new(Signed832.ExtendValue(Signed192.Signed(1)), Signed832.ExtendValue(Signed192.Signed(2)), one, one);
            if (native) Check(new NativePrismYawFrame3d(rectangleOrigin, Fixed64.Zero));
            else Check(ScaledRigidFrame3d.Rigid(rectangleOrigin, FixedQuaternion.Identity));

            void Check<TFrame>(TFrame target) where TFrame : struct, IForwardHullFrame3d
            {
                Assert.True(WideOrientedBox.TryGetConvexHullPenetration(source, Cube, Faces, Edges,
                    target, prism, Faces, Edges, out var selected));
                foreach (bool admissionOnly in new[] { true, false })
                {
                    var rows = new List<FixedContactAnchors>();
                    int count = WideOrientedBox.VisitProjectedHullRectangleContacts(source, origin, Cube, Faces, Edges,
                        target, rectangleOrigin, rectangle, clip, selected, rows,
                        static (pool, contact) => pool.Add(contact), admissionOnly);
                    if (domain is 0 or 4 or 5) { Assert.Equal(0, count); Assert.Empty(rows); continue; }
                    Assert.True(count > 0);
                    if (admissionOnly) { Assert.Empty(rows); continue; }
                    var corners = new HashSet<Vector3d>();
                    foreach (FixedContactAnchors row in rows)
                    {
                        Assert.Equal(Vector3d.Backward, row.Normal); Assert.Equal(extent, row.Depth);
                        Assert.True(row.FirstAnchor.TryGetPoint(out Vector3d moving));
                        Assert.True(row.SecondAnchor.TryGetPoint(out Vector3d wall));
                        Assert.Equal(-extent, moving.Z); Assert.Equal(Fixed64.Zero, wall.Z);
                        Assert.Equal(moving.X, wall.X); Assert.Equal(moving.Y, wall.Y);
                        Assert.InRange(wall.Y.m_rawValue, (-FixedMath.Min(height, extent)).m_rawValue, FixedMath.Min(height, extent).m_rawValue);
                        Fixed64 minimum = domain == 2 ? -Fixed64.Quarter : domain == 3 ? -extent : Fixed64.Zero;
                        Fixed64 maximum = domain == 2 ? Fixed64.Quarter : extent;
                        Assert.InRange(wall.X.m_rawValue, minimum.m_rawValue, maximum.m_rawValue);
                        corners.Add(wall);
                    }
                    if (domain == 2)
                    {
                        Assert.Contains(new Vector3d(-Fixed64.Quarter, -Fixed64.Quarter, Fixed64.Zero), corners);
                        Assert.Contains(new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), corners);
                    }
                }
            }
        }
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(3, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(3, true)]
    public void ExactScaledHull_ShouldPreserveRigidSatClassificationAndDepth(int separation, bool deduplicate)
    {
        FixedQuaternion firstRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)31);
        FixedQuaternion secondRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)(-11), (Fixed64)7, (Fixed64)19);
        Vector3d firstOrigin = new(-2, 1, 3), secondOrigin = firstOrigin + Vector3d.Right * separation;
        ScaledRigidFrame3d first = Frame(firstOrigin, firstRotation), second = Frame(secondOrigin, secondRotation);
        bool expected = WideOrientedBox.TryGetConvexHullPenetration(firstOrigin, firstRotation, Cube, Faces, Edges,
            secondOrigin, secondRotation, Cube, Faces, Edges, out var rigid, deduplicate);
        Assert.Equal(expected, WideOrientedBox.TryGetConvexHullPenetration(first, Cube, Faces, Edges,
            second, Cube, Faces, Edges, out var scaled, deduplicate));
        Assert.Equal(rigid.HasValue, scaled.HasValue);
        if (!expected) return;
        Assert.Equal(rigid.GetRoundedDepth(out bool rigidClamped), scaled.GetRoundedDepth(out bool scaledClamped));
        Assert.Equal(rigidClamped, scaledClamped);
        WideAxis3 oriented = rigid.Negate ? -rigid.Axis : rigid.Axis;
        Assert.Equal(WideNormalization.GetNormalized(Signed576.ExtendValue(oriented.X),
            Signed576.ExtendValue(oriented.Y), Signed576.ExtendValue(oriented.Z)), scaled.GetNormal());
    }

    [Fact]
    public void SignedAdmission_ShouldKeepBothExactExitsAndLeaveOverlapIndependentOfAdmission()
    {
        ScaledRigidFrame3d first = Frame(Vector3d.Zero, FixedQuaternion.Identity);
        ScaledRigidFrame3d second = Frame(Vector3d.Right, FixedQuaternion.Identity);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(first, Cube, Faces, Edges,
            second, Cube, Faces, Edges, out var rejected, true, Reject));
        Assert.False(rejected.HasValue);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(first, Cube, Faces, Edges,
            second, Cube, Faces, Edges, out var admitted, true, AdmitNegativeX));
        Assert.Equal(-Vector3d.Right, admitted.GetNormal());
        Assert.Equal((Fixed64)3, admitted.GetRoundedDepth(out bool clamped));
        Assert.False(clamped);
        static bool Reject(in WideScaledHullPenetration _) => false;
        static bool AdmitNegativeX(in WideScaledHullPenetration value) => value.Axis.Y.IsZero
            && value.Axis.Z.IsZero && value.Axis.X.Sign * (value.Negate ? -1 : 1) < 0;
    }

    [Theory]
    [InlineData(0, true)]
    [InlineData(3, false)]
    public void FractionallyScaledPlanarSheets_ShouldRetainTheirFiniteInPlaneSeparators(int separation, bool overlap)
    {
        Vector3d[] quad = { new(-1, 0, -1), new(1, 0, -1), new(1, 0, 1), new(-1, 0, 1) };
        int[] triangles = { 0, 1, 2, 0, 2, 3 }, edges = { 0, 1, 1, 2, 2, 3, 3, 0 };
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)45);
        Vector3d scale = new((Fixed64)2 / 3, Fixed64.One, (Fixed64)3 / 5);
        ScaledRigidFrame3d first = new(Vector3d.Zero, rotation, Vector3d.Zero, scale, rotation, Vector3d.Zero, Vector3d.One);
        ScaledRigidFrame3d second = new(Vector3d.Right * separation, rotation, Vector3d.Zero, scale,
            rotation, Vector3d.Zero, Vector3d.One);
        Assert.Equal(overlap, WideOrientedBox.TryGetConvexHullPenetration(first, quad, triangles, edges,
            second, quad, triangles, edges, out var result, true, planarPair: true));
        if (overlap) Assert.Equal(Fixed64.Zero, result.GetRoundedDepth(out _));
    }

    [Theory]
    [InlineData(0, 0)]
    [InlineData(2, 0)]
    [InlineData(3, 0)]
    [InlineData(4, 0)]
    [InlineData(5, 0)]
    [InlineData(0, 2)]
    [InlineData(3, 2)]
    [InlineData(4, 2)]
    [InlineData(5, 2)]
    public void ScaledHullCapsule_ShouldRetainRigidReferenceAdmissionDepthAndStrictOverlap(int offset, int length)
    {
        Vector3d[] authored = new Vector3d[Cube.Length], physical = new Vector3d[Cube.Length];
        for (int index = 0; index < Cube.Length; index++)
        { authored[index] = Cube[index] * 4; physical[index] = Cube[index] * 3; }
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * ((Fixed64)3 / 4), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var center = new Vector3d(offset, 0, 0);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)31);
        bool expected = WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(Vector3d.Zero,
            FixedQuaternion.Identity, physical, Faces, Edges, center, rotation, Vector3d.Up,
            (Fixed64)length, Fixed64.One, out var reference);
        Assert.Equal(expected, WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(frame, Vector3d.Zero,
            authored, Faces, Edges, center, rotation, Vector3d.Up, (Fixed64)length, Fixed64.One, out var contact));
        Assert.Equal(WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(Vector3d.Zero,
            FixedQuaternion.Identity, physical, Faces, Edges, center, rotation, Vector3d.Up,
            (Fixed64)length, Fixed64.One), WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(frame,
            authored, Faces, Edges, center, rotation, Vector3d.Up, (Fixed64)length, Fixed64.One));
        if (!expected) return;
        Assert.Equal(reference.Depth, contact.Depth);
        Assert.Equal(reference.Normal, contact.Normal);
        Assert.Equal(reference.DepthIsClamped, contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out var hullPoint));
        Assert.True(contact.SecondAnchor.TryGetPoint(out _));
        Assert.InRange(hullPoint.X.m_rawValue, ((Fixed64)(-3)).m_rawValue, ((Fixed64)3).m_rawValue);
        Assert.InRange(hullPoint.Y.m_rawValue, ((Fixed64)(-3)).m_rawValue, ((Fixed64)3).m_rawValue);
        Assert.InRange(hullPoint.Z.m_rawValue, ((Fixed64)(-3)).m_rawValue, ((Fixed64)3).m_rawValue);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactHullFiniteManifold_ShouldRetainFiniteFaceVerticesOrCrossedEdgeFeet(bool crossed)
    {
        Vector3d[] first=Array.ConvertAll(Cube,p=>new Vector3d(p.X,p.Y*(crossed?2:1),p.Z*(crossed?Fixed64.Quarter:Fixed64.One))*4);
        Vector3d[] second=Array.ConvertAll(Cube,p=>new Vector3d(p.X,p.Y*(crossed?Fixed64.Quarter:Fixed64.Two),p.Z*2)*4);
        Vector3d offset=Vector3d.Right*((Fixed64)3/2);
        ScaledRigidFrame3d a=ScaledFrame(Vector3d.Zero),b=ScaledFrame(offset);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(a,first,Faces,Edges,b,second,Faces,Edges,out var penetration));
        var contacts=new List<FixedContactAnchors>();
        Assert.True(WideOrientedBox.VisitConvexHullContacts(a,Vector3d.Zero,first,Faces,Edges,b,offset,second,Faces,Edges,
            penetration,contacts,static(pool,contact)=>pool.Add(contact))>0);
        foreach(var contact in contacts)
        {
            Assert.Equal(Vector3d.Right,contact.Normal);Assert.Equal(Fixed64.Half,contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));Assert.Equal(Fixed64.One,point.X);
            Assert.Equal(crossed?Fixed64.Quarter:Fixed64.One,point.Y.Abs());
            Assert.Equal(crossed?Fixed64.Quarter:Fixed64.One,point.Z.Abs());
        }
        Assert.Equal(1,WideOrientedBox.VisitConvexHullContacts(a,Vector3d.Zero,first,Faces,Edges,b,offset,second,Faces,Edges,
            penetration,0,static(_,_)=>throw new InvalidOperationException("Admission must retain the witness without materializing it."),admissionOnly:true));
        if(!crossed)
        {
            Assert.True(WideOrientedBox.TryGetConvexHullPenetration(b,second,Faces,Edges,a,first,Faces,Edges,out var reversed));
            contacts.Clear();
            Assert.True(WideOrientedBox.VisitConvexHullContacts(b,offset,second,Faces,Edges,a,Vector3d.Zero,first,Faces,Edges,
                reversed,contacts,static(pool,contact)=>pool.Add(contact))>0);
            foreach(var contact in contacts)
            {
                Assert.Equal(Vector3d.Left,contact.Normal);Assert.Equal(Fixed64.Half,contact.Depth);
                Assert.True(contact.FirstAnchor.TryGetPoint(out var point));Assert.Equal(Fixed64.Half,point.X);
                Assert.True(contact.SecondAnchor.TryGetPoint(out point));Assert.Equal(Fixed64.One,point.X);
            }
        }
        foreach(bool empty in new[]{false,true})
        foreach(bool admissionOnly in new[]{false,true})
        {
            var domain=new RetainedSupportDomain(empty,false);
            Assert.Equal(0,WideOrientedBox.VisitConvexHullContacts(a,Vector3d.Zero,first,Faces,Edges,b,offset,second,Faces,Edges,
                penetration,0,static(_,_)=>throw new InvalidOperationException("An excluded support cannot emit a contact."),domain,admissionOnly));
        }
    }

    [Fact]
    public void ExactHullRetainedEdges_ShouldMaterializeTheirAdmittedInteriorCuts()
    {
        Vector3d[] first=Array.ConvertAll(Cube,p=>p*4),second=Array.ConvertAll(Cube,p=>new Vector3d(p.X,p.Y*2,p.Z*2)*4);
        Vector3d offset=Vector3d.Right*((Fixed64)3/2);ScaledRigidFrame3d a=ScaledFrame(Vector3d.Zero),b=ScaledFrame(offset);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(a,first,Faces,Edges,b,second,Faces,Edges,out var penetration));
        var contacts=new List<FixedContactAnchors>();var domain=new RetainedSupportDomain(false,true);
        Assert.True(WideOrientedBox.VisitConvexHullContacts(a,Vector3d.Zero,first,Faces,Edges,b,offset,second,Faces,Edges,
            penetration,contacts,static(pool,contact)=>pool.Add(contact),domain)>=2);
        foreach(var contact in contacts)
        {
            Assert.Equal(Fixed64.Half,contact.Depth);Assert.Equal(Vector3d.Right,contact.Normal);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d firstPoint));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d secondPoint));
            Assert.Equal(Fixed64.One,firstPoint.X);Assert.Equal(Fixed64.Half,secondPoint.X);
            Assert.Equal(firstPoint.Y,secondPoint.Y);Assert.Equal(firstPoint.Z,secondPoint.Z);
            Assert.True(firstPoint.Y.Abs()==Fixed64.Half||firstPoint.Z.Abs()==Fixed64.Half);
        }
    }

    [Fact]
    public void ExactNonparallelSupportEdges_ShouldRetainOrDeclineTheirRationalIntersection()
    {
        Vector3d[] first={new(-4,0,0),new(4,0,0),new(0,4,-4)};
        Vector3d[] second={new(0,-4,0),new(0,4,0),new(4,0,4)};
        int[] faces={0,1,2},edges={0,1,1,2,2,0};var frame=ScaledFrame(Vector3d.Zero);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(frame,first,faces,edges,frame,second,faces,edges,
            out var penetration,admission:static(in WideScaledHullPenetration value)=>
            {
                WideDirection3d normal=value.Negate?-value.Axis:value.Axis;
                return normal.X.IsZero&&normal.Y.IsZero&&normal.Z.Sign>0;
            }));
        var contacts=new List<FixedContactAnchors>();
        Assert.Equal(1,WideOrientedBox.VisitConvexHullContacts(frame,Vector3d.Zero,first,faces,edges,frame,Vector3d.Zero,
            second,faces,edges,penetration,contacts,static(pool,contact)=>pool.Add(contact)));
        var contact=Assert.Single(contacts);Assert.Equal(Fixed64.Zero,contact.Depth);Assert.Equal(Vector3d.Forward,contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out var a));Assert.Equal(Vector3d.Zero,a);
        Assert.True(contact.SecondAnchor.TryGetPoint(out var b));Assert.Equal(a,b);
        foreach(bool admissionOnly in new[]{false,true})
        foreach(bool acceptFirst in new[]{false,true})
            Assert.Equal(0,WideOrientedBox.VisitConvexHullContacts(frame,Vector3d.Zero,first,faces,edges,frame,Vector3d.Zero,
                second,faces,edges,penetration,0,static(_,_)=>throw new InvalidOperationException("Excluded edge feet cannot emit."),
                new RetainedSupportDomain(false,false,acceptFirst),admissionOnly));
        Assert.Equal(1,WideOrientedBox.VisitConvexHullContacts(frame,Vector3d.Zero,first,faces,edges,frame,Vector3d.Zero,
            second,faces,edges,penetration,0,static(_,_)=>throw new InvalidOperationException("Admission must not materialize."),admissionOnly:true));
        Vector3d far=new(Fixed64.MinValue,Fixed64.Zero,Fixed64.Zero);
        Assert.Equal(0,WideOrientedBox.VisitConvexHullContacts(frame,far,first,faces,edges,frame,Vector3d.Zero,
            second,faces,edges,penetration,0,static(_,_)=>throw new InvalidOperationException("Unrepresentable source feet cannot emit.")));
        Assert.Equal(0,WideOrientedBox.VisitConvexHullContacts(frame,Vector3d.Zero,first,faces,edges,frame,far,
            second,faces,edges,penetration,0,static(_,_)=>throw new InvalidOperationException("Unrepresentable target feet cannot emit.")));
    }

    [Fact]
    public void ScaledBoxHullContact_ShouldRetainSupportVerticesSeparationAndRigidFastPath()
    {
        Vector3d[] authored=Array.ConvertAll(Cube,p=>p*4);var frame=ScaledFrame(Vector3d.Zero);
        Assert.True(WideOrientedBox.TryGetConvexHullContact(Vector3d.Right*((Fixed64)3/2),FixedQuaternion.Identity,
            Vector3d.One,frame,Vector3d.Zero,authored,Faces,Edges,out var contact));
        Assert.Equal(Vector3d.Left,contact.Normal);Assert.Equal(Fixed64.Half,contact.Depth);
        Assert.False(WideOrientedBox.TryGetConvexHullContact(Vector3d.Right*4,FixedQuaternion.Identity,
            Vector3d.One,frame,Vector3d.Zero,authored,Faces,Edges,out _));
        var rigid=Frame(Vector3d.Zero,FixedQuaternion.Identity);
        Assert.True(WideOrientedBox.TryGetConvexHullContact(Vector3d.Right,FixedQuaternion.Identity,Vector3d.One,
            rigid,Vector3d.Zero,Cube,Faces,Edges,out _));
        Vector3d[] collapsed=new Vector3d[Cube.Length];
        Assert.False(WideOrientedBox.TryGetConvexHullPenetration(frame,collapsed,Faces,Edges,frame,collapsed,Faces,Edges,out _));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ScaledHullCapsule_ParallelCoreBeyondFaceMidpoint_ShouldMatchTheFiniteSupportSet(int axis)
    {
        Vector3d[] authored=Array.ConvertAll(Cube,p=>p*4),physical=Array.ConvertAll(Cube,p=>p*3);
        Vector3d normal=axis==0?Vector3d.Right:axis==1?Vector3d.Up:Vector3d.Forward;
        Vector3d coreAxis=axis==0?Vector3d.Up:axis==1?Vector3d.Forward:Vector3d.Right;
        Vector3d center=normal*((Fixed64)7/2)+coreAxis*4;
        var frame=new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            Vector3d.One*((Fixed64)3/4),FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(frame,Vector3d.Zero,authored,Faces,Edges,
            center,FixedQuaternion.Identity,coreAxis,(Fixed64)6,Fixed64.One,out var contact));
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(Vector3d.Zero,FixedQuaternion.Identity,
            physical,Faces,Edges,center,FixedQuaternion.Identity,coreAxis,(Fixed64)6,Fixed64.One,out var reference));
        Assert.Equal(reference.Normal,contact.Normal);Assert.Equal(reference.Depth,contact.Depth);
        Assert.Equal(normal,contact.Normal);Assert.Equal(Fixed64.Half,contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out var hull));Assert.True(contact.SecondAnchor.TryGetPoint(out var capsule));
        Assert.Equal((Fixed64)3,Vector3d.Dot(hull,normal));
        Assert.Equal((Fixed64)5/2,Vector3d.Dot(capsule,normal));
        Fixed64 coordinate=Vector3d.Dot(hull,coreAxis);
        Assert.InRange(coordinate.m_rawValue,Fixed64.One.m_rawValue,((Fixed64)3).m_rawValue);
        Assert.Equal(coordinate,Vector3d.Dot(capsule,coreAxis));
        var rigid=Frame(Vector3d.Zero,FixedQuaternion.Identity);
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(rigid,Vector3d.Zero,physical,Faces,Edges,
            center,FixedQuaternion.Identity,coreAxis,(Fixed64)6,Fixed64.One,out _));
        Assert.True(WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(rigid,physical,Faces,Edges,
            center,FixedQuaternion.Identity,coreAxis,(Fixed64)6,Fixed64.One));
    }

    [Fact]
    public void ScaledHullRepresentative_ShouldRetainBothSignedExitsAndRejectUnrepresentableTranslatedWitnesses()
    {
        Vector3d[] authored=Array.ConvertAll(Cube,p=>p*4);
        var first=ScaledFrame(Vector3d.Zero);var second=ScaledFrame(Vector3d.Right*((Fixed64)3/2));
        Assert.True(WideOrientedBox.TryGetConvexHullContact(first,Vector3d.Zero,authored,Faces,Edges,
            second,Vector3d.Zero,authored,Faces,Edges,out var contact));
        Assert.Equal(Vector3d.Right,contact.Normal);Assert.Equal(Fixed64.Half,contact.Depth);
        Assert.Equal(Fixed64.One,contact.FirstAnchor.LocalPoint.X);
        Assert.Equal(Fixed64.Half,contact.SecondAnchor.LocalPoint.X);
        Vector3d outside=new(Fixed64.MinValue,Fixed64.Zero,Fixed64.Zero);
        Assert.False(WideOrientedBox.TryGetConvexHullContact(first,outside,authored,Faces,Edges,
            second,Vector3d.Zero,authored,Faces,Edges,out _));
        Assert.False(WideOrientedBox.TryGetConvexHullContact(first,Vector3d.Zero,authored,Faces,Edges,
            second,outside,authored,Faces,Edges,out _));
    }

    [Fact]
    public void ScaledTriangleHulls_ShouldRetainAnEdgeCrossSeparatingAxis()
    {
        Vector3d[] first={new(4,-8,8),new(0,12,4),new(-8,8,4)};
        Vector3d[] second={new(8,8,-8),new(-8,0,-12),new(-8,-4,8)};
        int[] faces={0,1,2},edges={0,1,1,2,2,0};var frame=ScaledFrame(Vector3d.Zero);
        Assert.False(WideOrientedBox.TryGetConvexHullPenetration(frame,first,faces,edges,
            frame,second,faces,edges,out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ScaledHullRoundedCorner_ShouldRetainTheFiniteEdgeOrVertexSupport(bool vertex)
    {
        Vector3d[] authored=Array.ConvertAll(Cube,p=>p*4),physical=Array.ConvertAll(Cube,p=>p*3);
        var frame=new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            Vector3d.One*((Fixed64)3/4),FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        Vector3d center=new((Fixed64)7/2,(Fixed64)7/2,vertex?(Fixed64)7/2:Fixed64.Zero);
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(frame,Vector3d.Zero,authored,Faces,Edges,
            center,FixedQuaternion.Identity,Vector3d.Up,Fixed64.Zero,Fixed64.One,out var contact));
        var cycledFaces=new int[Faces.Length];
        for(int index=0;index<Faces.Length;index+=3)
        {cycledFaces[index]=Faces[index+1];cycledFaces[index+1]=Faces[index+2];cycledFaces[index+2]=Faces[index];}
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(frame,Vector3d.Zero,authored,cycledFaces,Edges,
            center,FixedQuaternion.Identity,Vector3d.Up,Fixed64.Zero,Fixed64.One,out var cycled));
        Assert.Equal(contact.Normal,cycled.Normal);Assert.Equal(contact.Depth,cycled.Depth);
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(Vector3d.Zero,FixedQuaternion.Identity,
            physical,Faces,Edges,center,FixedQuaternion.Identity,Vector3d.Up,Fixed64.Zero,Fixed64.One,out var reference));
        Assert.Equal(reference.Normal,contact.Normal);Assert.Equal(reference.Depth,contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d hull));
        Assert.Equal(new Vector3d((Fixed64)3,(Fixed64)3,vertex?(Fixed64)3:Fixed64.Zero),hull);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d round));
        Assert.True(Vector3d.Dot(round-hull,contact.Normal)<=Fixed64.Zero);
    }

    private sealed class RetainedSupportDomain:IScaledConvexHullContactDomain
    {
        private static readonly RationalSegmentInterval[] Full={default};
        private static readonly RationalSegmentInterval[] Middle={new(Signed320.ExtendValue(Signed192.Signed(1)),
            Signed320.ExtendValue(Signed192.Signed(4)),Signed320.ExtendValue(Signed192.Signed(3)),Signed320.ExtendValue(Signed192.Signed(4)))};
        private readonly bool empty,retainMiddle,acceptFirst;
        internal RetainedSupportDomain(bool empty,bool retainMiddle,bool acceptFirst=false)
        {this.empty=empty;this.retainMiddle=retainMiddle;this.acceptFirst=acceptFirst;}
        public ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first,int start,int end,in WideDirection3d normal)=>
            empty?ReadOnlySpan<RationalSegmentInterval>.Empty:retainMiddle?Middle:Full;
        public bool ContainsPoint(bool first,int start,int end,in Signed832 n,in Signed832 d,in WideDirection3d normal)=>
            acceptFirst&&first||retainMiddle&&start!=end&&RationalSegmentInterval.CompareParameters(n,d,Middle[0].WideLowerNumerator,Middle[0].WideLowerDenominator)>=0
                &&RationalSegmentInterval.CompareParameters(n,d,Middle[0].WideUpperNumerator,Middle[0].WideUpperDenominator)<=0;
        public bool ContainsPoint(bool first,int start,int end,scoped ContactQuadratic n,scoped ContactQuadratic d,in WideDirection3d normal)=>
            acceptFirst&&first||retainMiddle&&start!=end&&Middle[0].Contains(n,d,ReadOnlySpan<ulong>.Empty);
    }

    private static ScaledRigidFrame3d ScaledFrame(Vector3d translation)=>new(translation,FixedQuaternion.Identity,
        Vector3d.Zero,Vector3d.One/4,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);

    private static ScaledRigidFrame3d Frame(Vector3d origin, FixedQuaternion rotation) =>
        new(origin, rotation, Vector3d.Zero, Vector3d.One, rotation, Vector3d.Zero, Vector3d.One);
}
