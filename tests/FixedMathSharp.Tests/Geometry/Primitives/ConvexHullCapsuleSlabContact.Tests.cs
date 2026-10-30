using System;
using System.Runtime.CompilerServices;
using System.Threading;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class ConvexHullCapsuleSlabContactTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PerpendicularFaceSeed_KeepsTheFiniteSeamWithinItsProvenNormalWidth(bool affine)
    {
        Vector3d[] vertices = Cube(new Vector3d(Fixed64.Quarter, (Fixed64)2048, (Fixed64)2048));
        const long q = 1920767767;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.FromRaw(q), Fixed64.Zero, Fixed64.FromRaw(2 * q));
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, rotation, Vector3d.Zero,
            Vector3d.One * (affine ? Fixed64.One + Fixed64.MinIncrement : Fixed64.One), rotation, Vector3d.Zero,
            Vector3d.One * (affine ? Fixed64.One + Fixed64.FromRaw(3) : Fixed64.One));
        Assert.True(frame.TryGetPoint(new Vector3d((Fixed64).125m, Fixed64.Zero, Fixed64.Half), out Vector3d center));
        Fixed64 length = (Fixed64)1.5m + Fixed64.FromRaw(3), radius = Fixed64.Quarter + Fixed64.MinIncrement;
        var hull = new CylinderHullSupport(frame, vertices, center, rotation, Fixed64.Half, radius);
        ReadOnlySpan<int> triangles = WideOrientedBox.BoxHullTriangles;
        int index = 0;
        while (!(vertices[triangles[index]].X > Fixed64.Zero
            && vertices[triangles[index]].X == vertices[triangles[index + 1]].X
            && vertices[triangles[index]].X == vertices[triangles[index + 2]].X)) index += 3;
        var triangle = new FixedTriangle(vertices[triangles[index]], vertices[triangles[index + 1]], vertices[triangles[index + 2]]);
        var geometry = new TriangleCircularGeometry(triangle, frame, hull, triangles[index], triangles[index + 1], triangles[index + 2])
            .WithCore(Vector2d.Forward, length, out WideDirection3d coreOffset);
        geometry = geometry.WithValueShift(hull.GetCoreValueShift(geometry, coreOffset));
        Span<ulong> magnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(geometry.FaceNormal.X, magnitude);
        Assert.True(WideArithmetic.GetMagnitudeBitLength(magnitude) > (affine ? 344 : 230));
        WideDirection3d axis = new(default, default, Signed576.ExtendValue(Signed192.Raw(Fixed64.One)));
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * CylinderContactAlgebra.Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> normal = stackalloc ulong[3 * CylinderContactAlgebra.Words]; Span<int> normalSigns = stackalloc int[3];
        Assert.True(TriangleCylinderAnalyticFeatures.TryGetSeed(geometry, coreOffset, axis, values, signs, normal, normalSigns,
            out int gap, out int mask, out int coreSign, hull, includeSeam: false));
        Assert.Equal(0, coreSign);
        for (int component = 0; component < 3; component++)
            Assert.True(WideArithmetic.GetMagnitudeBitLength(normal.Slice(component * CylinderContactAlgebra.Words,
                CylinderContactAlgebra.Words)) <= (affine ? 344 : 230));
        geometry.GetAnalyticDepth(new ConvexContactCandidate(values, signs, gap), normalSigns, mask, out Fixed64 depth, out _);
        Assert.True(FixedMath.Abs(depth - (radius + (Fixed64).125m)) <= Fixed64.FromRaw(16));
        Vector3d radial = default;
        Assert.True(TriangleCapsuleSlabWitnesses.TryGetPoints(geometry, triangle, mask, normal, normalSigns, coreOffset,
            Vector2d.Forward, length, ref radial, out Vector3d hullPoint, out Vector3d corePoint, out FixedPointAnchorTerm3d term));
        var contact = new FixedContactAnchors(new FixedPointAnchor(frame.IsRigid ? frame.RigidOrigin : center,
            frame.IsRigid ? frame.Rotation : rotation, hullPoint), new FixedPointAnchor(center, rotation, corePoint, radial, term),
            WideConvexPrismRelations.GetConvexContactCandidateNormal(new ConvexContactCandidate(values, signs, gap)), depth, false);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Assert.True((offset - contact.Normal * depth).Magnitude <= Fixed64.FromRaw(64));
        AssertSlabWitness(contact, center, rotation, Vector2d.Forward, length, radius, Fixed64.Quarter);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void AnalyticEndpoint_PreservesOddCoreAndDegenerateRadius(bool affine, bool zeroRadius)
    {
        Vector3d[] vertices = Cube(Vector3d.One * (affine ? Fixed64.Half : Fixed64.One));
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            affine ? Vector3d.One * Fixed64.Two : Vector3d.One, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Fixed64 length = Fixed64.Half + Fixed64.MinIncrement, radius = zeroRadius ? Fixed64.Zero : Fixed64.Quarter;
        int[] triangles = WideOrientedBox.BoxHullTriangles.ToArray();
        foreach (bool reverse in new[] { false, true })
        {
            if (reverse) Array.Reverse(triangles);
            foreach (Fixed64 z in new[] { (Fixed64).75m, Fixed64.One })
            {
                Vector3d center = Vector3d.Forward * z;
                Assert.True(ConvexHullCylinderContact.TryGetContactWithContainedCenter(frame, vertices, triangles,
                    center, FixedQuaternion.Identity, Fixed64.Half, radius, out FixedContactAnchors contact, Vector2d.Forward, length));
                Assert.Equal(Vector3d.Forward, contact.Normal);
                Assert.True(FixedMath.Abs(contact.Depth - (Fixed64.One - z + Fixed64.Quarter + radius)) <= Fixed64.MinIncrement);
                Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
                Assert.True((offset - contact.Normal * contact.Depth).Magnitude <= Fixed64.FromRaw(4));
                AssertSlabWitness(contact, center, FixedQuaternion.Identity, Vector2d.Forward, length, radius, Fixed64.Quarter);
            }
        }
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void SeamWitness_DeclinesFeaturesOutsideTheFiniteSlab(int kind, bool affine)
    {
        FixedTriangle triangle = kind == 1
            ? new FixedTriangle(new Vector3d(3, 1, 3), new Vector3d(4, 1, 3), new Vector3d(3, 1, 4))
            : kind == 0 ? new FixedTriangle(new Vector3d(1, 3, 3), new Vector3d(1, 4, 3), new Vector3d(1, 3, 4))
            : new FixedTriangle(new Vector3d(1, 3, 3), new Vector3d(2, 2, 3), new Vector3d(1, 3, 4));
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            affine ? Vector3d.One * Fixed64.Two : Vector3d.One, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        if (affine) triangle = new FixedTriangle(triangle.A * Fixed64.Half, triangle.B * Fixed64.Half, triangle.C * Fixed64.Half);
        var geometry = new TriangleCircularGeometry(triangle, frame, Vector3d.Zero, FixedQuaternion.Identity,
            Signed192.Raw(Fixed64.Half), Fixed64.Quarter).WithCore(Vector2d.Forward, Fixed64.Half, out WideDirection3d coreOffset);
        Span<ulong> normal = stackalloc ulong[3 * CylinderContactAlgebra.Words]; Span<int> signs = stackalloc int[3];
        CylinderContactAlgebra.WriteDirection(new WideDirection3d(kind == 1 ? default : Signed576.One,
            kind == 0 ? default : Signed576.One, default), normal, signs);
        Vector3d radial = default;
        Assert.False(TriangleCapsuleSlabWitnesses.TryGetPoints(geometry, triangle, 7, normal, signs, coreOffset,
            Vector2d.Forward, Fixed64.Half, ref radial, out _, out _, out _));
    }

    [Fact]
    public void CollapsedFace_CannotSeedOrCertifyAnExtrudedHullMinimum()
    {
        var frame = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        Vector3d[] vertices = Cube(Vector3d.One);
        var hull = new CylinderHullSupport(frame, vertices, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Half, Fixed64.Quarter);
        var triangle = new FixedTriangle(vertices[0], vertices[0], vertices[0]);
        var chart = new TriangleCircularGeometry(triangle, frame, hull, 0, 0, 0)
            .WithCore(Vector2d.Forward, Fixed64.Half, out WideDirection3d coreOffset);
        WideDirection3d axis = new(default, default, Signed576.ExtendValue(Signed192.Raw(Fixed64.One)));
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * CylinderContactAlgebra.Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> direction = stackalloc ulong[3 * CylinderContactAlgebra.Words]; Span<int> directionSigns = stackalloc int[3];
        Assert.False(TriangleCylinderAnalyticFeatures.TryGetSeed(chart, coreOffset, axis, values, signs, direction, directionSigns,
            out _, out _, out _, hull, includeSeam: false));
        values.Clear(); signs.Clear();
        Assert.False(hull.HasExtrudedBallMinimumCertificate(frame, new[] { 0, 0, 0 }, chart, coreOffset, axis,
            new ConvexContactCandidate(values, signs, 0)));
        Assert.False(hull.HasExtrudedBallMinimumCertificate(frame, new[] { 0, 3, 4 }, chart, coreOffset, axis,
            new ConvexContactCandidate(values, signs, 0)));
        Assert.False(hull.HasExtrudedBallMinimumCertificate(frame, new[] { 0, 3, 6 }, chart, coreOffset, axis,
            new ConvexContactCandidate(values, signs, 0)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoreRimRoot_PreservesDirtyCallerHeadroomOnOneMiBStack(bool affine)
    {
        var (frame, vertices, center, rotation, length) = RootScene(affine);
        var contacts = new FixedContactAnchors[2];
        Exception? failure = null;
        var worker = new Thread(() =>
        {
            try
            {
                for (int pass = 0; pass < 2; pass++)
                    contacts[pass] = WithDirtyCaller(frame, vertices, center, rotation, length,
                        pass == 0 ? 0xA55A0FF012345678UL : 0x5AA5F00FFEDCBA98UL);
            }
            catch (Exception exception) { failure = exception; }
        }, 1024 * 1024);
        worker.Start(); worker.Join();
        Assert.Null(failure);
        Assert.Equal(contacts[0], contacts[1]);
        Assert.True(contacts[0].Depth > Fixed64.Zero);
    }

    [Fact]
    public void WideAuthoredAffineHull_PreservesFiniteSlabAndHullWitnesses()
    {
        Vector3d[] vertices = Cube(Vector3d.One * (Fixed64)1000000000);
        FixedQuaternion placement = new FixedQuaternion((Fixed64)1, (Fixed64)2, (Fixed64)3, (Fixed64)4).Normalized;
        FixedQuaternion orientation = new FixedQuaternion((Fixed64)3, (Fixed64)1, (Fixed64)4, (Fixed64)2).Normalized;
        var frame = new ScaledRigidFrame3d(new Vector3d(2, -1, 3), placement, Vector3d.Zero,
            new Vector3d(Fixed64.FromRaw(1), Fixed64.FromRaw(3), Fixed64.FromRaw(5)),
            orientation, Vector3d.Zero, new Vector3d(Fixed64.One + Fixed64.MinIncrement, (Fixed64).75m + Fixed64.MinIncrement, (Fixed64)1.25m));
        Assert.True(frame.TryGetPoint(Vector3d.Zero, out Vector3d center));
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, (Fixed64).4m);
        Fixed64 length = Fixed64.One + Fixed64.MinIncrement, radius = Fixed64.Half, height = Fixed64.Half;
        Assert.True(ConvexHullCylinderContact.TryGetContactWithContainedCenter(frame, vertices, WideOrientedBox.BoxHullTriangles,
            center, rotation, height, radius, out FixedContactAnchors contact, Vector2d.Forward, length));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Assert.True((offset - contact.Normal * contact.Depth).Magnitude <= Fixed64.FromRaw(64));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.True(contact.SecondAnchor.TryGetPoint(out _));
        AssertSlabWitness(contact, center, rotation, Vector2d.Forward, length, radius, height * Fixed64.Half);
        ReadOnlySpan<int> triangles = WideOrientedBox.BoxHullTriangles;
        bool onFace = false;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            Assert.True(frame.TryGetPoint(vertices[triangles[i]], out Vector3d a));
            Assert.True(frame.TryGetPoint(vertices[triangles[i + 1]], out Vector3d b));
            Assert.True(frame.TryGetPoint(vertices[triangles[i + 2]], out Vector3d c));
            if ((new FixedTriangle(a, b, c).ClosestPoint(point) - point).Magnitude <= Fixed64.FromRaw(64)) onFace = true;
        }
        Assert.True(onFace);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PositiveCoreRimRoot_PreservesTheWinningEndpointChart(bool affine)
    {
        var (frame, vertices, center, rotation, length) = RootScene(affine);
        Assert.True(ConvexHullCylinderContact.TryGetContact(frame, vertices, WideOrientedBox.BoxHullTriangles,
            center, rotation, (Fixed64)10, (Fixed64)25, out FixedContactAnchors contact, Vector2d.Forward, length));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Vector3d residual = offset - contact.Normal * contact.Depth;
        Fixed64 tolerance = Fixed64.MinIncrement * (Fixed64)32;
        Assert.True(FixedMath.Abs(residual.X) <= tolerance && FixedMath.Abs(residual.Y) <= tolerance
            && FixedMath.Abs(residual.Z) <= tolerance, residual.ToString());
        AssertSlabWitness(contact, center, rotation, Vector2d.Forward, length, (Fixed64)25, (Fixed64)5);
        foreach (int raw in new[] { -128, 128 })
        {
            Vector3d moved = center + contact.Normal * (contact.Depth + Fixed64.FromRaw(raw));
            var section = new FiniteSlabSectionFrame(frame, moved, rotation, length, (Fixed64)25, (Fixed64)5);
            bool intersects = false;
            ReadOnlySpan<int> triangles = WideOrientedBox.BoxHullTriangles;
            for (int index = 0; index < triangles.Length; index += 3)
                intersects |= section.IntersectsTriangle(new FixedTriangle(vertices[triangles[index]],
                    vertices[triangles[index + 1]], vertices[triangles[index + 2]]), ReadOnlySpan<Vector2d>.Empty);
            Assert.Equal(raw < 0, intersects);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IndependentlyPlacedHulls_ClearTheWholeSlabAtTheSelectedDepth(bool affine)
    {
        Vector3d[] vertices = Cube(new Vector3d((Fixed64)1.5m, (Fixed64).75m, Fixed64.One));
        for (int seed = 1; seed <= 24; seed++)
        {
            FixedQuaternion orientation = new FixedQuaternion((Fixed64)(seed % 4 + 1), (Fixed64)(seed % 3 + 1),
                (Fixed64)(seed % 5 - 2), (Fixed64)3).Normalized;
            var frame = new ScaledRigidFrame3d(new Vector3d(2, -1, 3), orientation, Vector3d.Zero,
                affine ? new Vector3d(Fixed64.One + Fixed64.MinIncrement, (Fixed64).75m, (Fixed64)1.25m) : Vector3d.One,
                FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
            Assert.True(frame.TryGetPoint(Vector3d.Zero, out Vector3d center));
            FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(seed % 7, 5));
            Fixed64 height = (Fixed64).75m, radius = (Fixed64).625m + Fixed64.MinIncrement;
            Fixed64 length = Fixed64.FromFraction(seed % 5 + 1, 2) + Fixed64.MinIncrement;
            Assert.True(ConvexHullCylinderContact.TryGetContactWithContainedCenter(frame, vertices, WideOrientedBox.BoxHullTriangles,
                center, rotation, height, radius, out FixedContactAnchors contact, Vector2d.Forward, length));
            Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
            Vector3d residual = offset - contact.Normal * contact.Depth;
            Fixed64 tolerance = Fixed64.MinIncrement * (Fixed64)32;
            Assert.True(FixedMath.Abs(residual.X) <= tolerance && FixedMath.Abs(residual.Y) <= tolerance
                && FixedMath.Abs(residual.Z) <= tolerance, $"seed {seed}: {residual}");
            AssertSlabWitness(contact, center, rotation, Vector2d.Forward, length, radius, height * Fixed64.Half);
            foreach (int raw in new[] { -128, 128 })
            {
                Vector3d moved = center + contact.Normal * (contact.Depth + Fixed64.FromRaw(raw));
                var section = new FiniteSlabSectionFrame(frame, moved, rotation, length, radius, height * Fixed64.Half);
                bool intersects = false;
                ReadOnlySpan<int> triangles = WideOrientedBox.BoxHullTriangles;
                for (int index = 0; index < triangles.Length; index += 3)
                    intersects |= section.IntersectsTriangle(new FixedTriangle(vertices[triangles[index]],
                        vertices[triangles[index + 1]], vertices[triangles[index + 2]]), ReadOnlySpan<Vector2d>.Empty);
                Assert.Equal(raw < 0, intersects);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LongPlanarCores_MatchIndependentCapsulePolygonDepth(bool affine)
    {
        Vector3d half = new(1, 4, 1);
        Vector3d[] vertices = Cube(affine ? half * Fixed64.Half : half);
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            affine ? Vector3d.One * Fixed64.Two : Vector3d.One, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Vector2d[] polygon = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        var slabAnchors = new FixedPointAnchor2d[2];
        var polygonAnchors = new FixedPointAnchor2d[2];
        for (int seed = 1; seed <= 24; seed++)
        {
            Vector2d axis = new Vector2d((Fixed64)(seed % 7 + 1), (Fixed64)(seed % 5 + 1)).Normalized;
            Vector2d center = new(Fixed64.FromFraction(seed % 3 - 1, 4), Fixed64.FromFraction(seed % 5 - 2, 8));
            Fixed64 length = (Fixed64)(seed % 5 + 4), radius = Fixed64.Quarter;
            Assert.True(WideCenteredCapsule2dRelations.TryGetContacts(center, Fixed64.Zero, axis, length, radius,
                Vector2d.Zero, Fixed64.Zero, polygon, slabAnchors, polygonAnchors, out _, out _, out Fixed64 expected, out _));
            Assert.True(ConvexHullCylinderContact.TryGetContactWithContainedCenter(frame, vertices, WideOrientedBox.BoxHullTriangles,
                new Vector3d(center.X, Fixed64.Zero, center.Y), FixedQuaternion.Identity, Fixed64.Half, radius,
                out FixedContactAnchors contact, axis, length));
            Assert.Equal(expected, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.True(FixedMath.Abs(first.X) <= half.X + Fixed64.MinIncrement);
            Assert.True(FixedMath.Abs(first.Y) <= half.Y + Fixed64.MinIncrement);
            Assert.True(FixedMath.Abs(first.Z) <= half.Z + Fixed64.MinIncrement);
            Vector3d residual = first - second - contact.Normal * contact.Depth;
            Fixed64 tolerance = Fixed64.MinIncrement * (Fixed64)8;
            Assert.True(FixedMath.Abs(residual.X) <= tolerance && FixedMath.Abs(residual.Y) <= tolerance
                && FixedMath.Abs(residual.Z) <= tolerance);
            AssertSlabWitness(contact, new Vector3d(center.X, Fixed64.Zero, center.Y), FixedQuaternion.Identity, axis, length, radius, Fixed64.Quarter);
        }
    }

    private static void AssertSlabWitness(FixedContactAnchors contact, Vector3d center, FixedQuaternion rotation,
        Vector2d axis, Fixed64 length, Fixed64 radius, Fixed64 halfThickness)
    {
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d point));
        Assert.True(rotation.TryInverseTransformScaledPoint(center, point, Vector3d.One, out Vector3d local));
        Fixed64 tolerance = Fixed64.FromRaw(128);
        Assert.True(FixedMath.Abs(local.Y) <= halfThickness + tolerance);
        Vector2d planar = new(local.X, local.Z);
        Fixed64 parameter = Vector2d.Dot(planar, axis) / Vector2d.Dot(axis, axis);
        parameter = FixedMath.Clamp(parameter, -length * Fixed64.Half, length * Fixed64.Half);
        Fixed64 distance = (planar - axis * parameter).Magnitude;
        Assert.True(distance <= radius + tolerance);
        Assert.True(FixedMath.Abs(local.Y) >= halfThickness - tolerance || distance >= radius - tolerance);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static FixedContactAnchors WithDirtyCaller(in ScaledRigidFrame3d frame, Vector3d[] vertices, Vector3d center,
        FixedQuaternion rotation, Fixed64 length, ulong seed)
    {
        Span<ulong> caller = stackalloc ulong[8192];
        for (int i = 0; i < caller.Length; i++) caller[i] = seed ^ unchecked(0x9E3779B97F4A7C15UL * (ulong)(i + 1));
        Assert.True(ConvexHullCylinderContact.TryGetContact(frame, vertices, WideOrientedBox.BoxHullTriangles,
            center, rotation, (Fixed64)10, (Fixed64)25, out FixedContactAnchors contact, Vector2d.Forward, length));
        for (int i = 0; i < caller.Length; i++) Assert.Equal(seed ^ unchecked(0x9E3779B97F4A7C15UL * (ulong)(i + 1)), caller[i]);
        return contact;
    }

    private static (ScaledRigidFrame3d, Vector3d[], Vector3d, FixedQuaternion, Fixed64) RootScene(bool affine)
    {
        Vector3d extents = new((Fixed64)10, Fixed64.Half, Fixed64.Half);
        Vector3d[] vertices = Cube(affine ? extents * Fixed64.Half : extents);
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            affine ? Vector3d.One * Fixed64.Two : Vector3d.One, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        const long q = 1920767767;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-q), Fixed64.FromRaw(2 * q));
        Vector3d center = new((Fixed64)(-5), (Fixed64)15.5m - Fixed64.FromFraction(15, 1024), (Fixed64)20.5m - Fixed64.FromFraction(16, 1024));
        return (frame, vertices, center, rotation, Fixed64.Half + Fixed64.FromRaw(3));
    }

    private static Vector3d[] Cube(Vector3d half) => new[]
    {
        new Vector3d(-half.X,-half.Y,-half.Z),new Vector3d(half.X,-half.Y,-half.Z),
        new Vector3d(-half.X,half.Y,-half.Z),new Vector3d(half.X,half.Y,-half.Z),
        new Vector3d(-half.X,-half.Y,half.Z),new Vector3d(half.X,-half.Y,half.Z),
        new Vector3d(-half.X,half.Y,half.Z),new Vector3d(half.X,half.Y,half.Z)
    };
}
