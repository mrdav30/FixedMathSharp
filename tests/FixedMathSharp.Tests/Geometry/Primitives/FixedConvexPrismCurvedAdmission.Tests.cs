using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismCurvedAdmissionTests
{
    private static readonly Vector2d[] Square = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
    // Common translation leaves the independent relative support gap unchanged.
    private static readonly Vector3d LargeTranslation = new(
        Fixed64.MaxValue - (Fixed64)16, Fixed64.MinValue + (Fixed64)16, Fixed64.MaxValue - (Fixed64)16);
    private static readonly Vector2d[] ReversedRedundantSquare =
    {
        new(-1, 1), new(1, 1), new(1, -1), new(1, -1), new(0, -1), new(-1, -1)
    };

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void ConeContact_RejectsWholeShapeSeparationAtSlabCorner(bool reverse, bool largeOrigin)
    {
        Vector3d center = new(Fixed64.FromRaw(-565576967), Fixed64.FromRaw(-557169159), Fixed64.FromRaw(-565576967));
        Vector3d normal = new(Fixed64.FromRaw(2716375826), Fixed64.FromRaw(1920767767), Fixed64.FromRaw(2716375826));
        Vector2d[] offsets = { new(-2, -2), new(2, -2), new(2, 2), new(-2, 2) };
        if (reverse)
            Array.Reverse(offsets);
        Fixed64 radial = FixedMath.Sqrt(normal.X * normal.X + normal.Z * normal.Z);
        Fixed64 maximum = Vector3d.Dot(center, normal) + FixedMath.Max(
            Fixed64.Half * normal.Y, Fixed64.Half * radial - Fixed64.Half * normal.Y);
        // Every slab coordinate is nonnegative, while the complete cone lies behind this plane.
        Assert.True(-maximum > Fixed64.FromFraction(1, 2048));
        Vector3d translation = largeOrigin ? LargeTranslation : Vector3d.Zero;

        Assert.False(FixedConvexPrismRelations.TryGetCenteredConeContact(
            center + translation, FixedQuaternion.Identity, Vector3d.Up, Fixed64.One, Fixed64.Half,
            new Vector3d(2, 1, 2) + translation, Fixed64.Zero, offsets, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CylinderContact_RejectsWholeShapeSeparationAtSlabRim(bool reverse, bool largeOrigin)
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)22, (Fixed64)17, (Fixed64)9);
        Vector3d normal = new(Fixed64.Zero, Fixed64.FromRaw(-3980925575), Fixed64.FromRaw(-1612133878));
        Vector3d center = new Vector3d(Fixed64.Quarter, Fixed64.FromFraction(3, 4), Fixed64.Quarter)
            - normal * (Fixed64.FromRaw(1918558518) + Fixed64.FromFraction(1, 1024));
        Vector2d[] offsets = { new(-Fixed64.One, -Fixed64.Half), new(Fixed64.One, -Fixed64.Half),
            new(Fixed64.One, Fixed64.Half), new(-Fixed64.One, Fixed64.Half) };
        if (reverse)
            Array.Reverse(offsets);
        Vector3d local = rotation.Conjugate().Rotate(normal);
        Fixed64 maximum = Vector3d.Dot(center, normal)
            + FixedMath.Sqrt(local.X * local.X + local.Z * local.Z) + local.Y.Abs();
        Fixed64 minimum = normal.Z * Fixed64.Half;
        Assert.True(minimum - maximum > Fixed64.FromFraction(1, 2048));
        Vector3d translation = largeOrigin ? LargeTranslation : Vector3d.Zero;

        Assert.False(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            center + translation, rotation, Vector3d.Up, Fixed64.Two, Fixed64.One,
            new Vector3d(0, -4, 0) + translation, Fixed64.Zero, offsets, (Fixed64)4, out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Theory]
    [InlineData("cone base", -1073741824L)]
    [InlineData("cone base", -1L)]
    [InlineData("cone base", 0L)]
    [InlineData("cone base", 1L)]
    [InlineData("cylinder cap", -1073741824L)]
    [InlineData("cylinder cap", -1L)]
    [InlineData("cylinder cap", 0L)]
    [InlineData("cylinder cap", 1L)]
    [InlineData("cylinder side", -1073741824L)]
    [InlineData("cylinder side", -1L)]
    [InlineData("cylinder side", 0L)]
    [InlineData("cylinder side", 1L)]
    public void CurvedFaces_PreserveClosedBoundaryDepthAndSurfaceWitnesses(string feature, long gapRaw)
    {
        bool cone = feature == "cone base";
        bool side = feature == "cylinder side";
        Vector3d normal = cone ? Vector3d.Down : side ? Vector3d.Right : Vector3d.Up;
        Vector3d prismOrigin = normal * (Fixed64.Two + Fixed64.FromRaw(gapRaw));
        foreach (Vector2d[] footprint in new[] { Square, ReversedRedundantSquare })
        {
            bool hit = TryContact(cone, Vector3d.Zero, Vector3d.Up, Fixed64.One,
                prismOrigin, Fixed64.Zero, footprint, Fixed64.One, out FixedContactAnchors contact);
            Assert.Equal(gapRaw <= 0, hit);
            if (!hit)
            {
                Assert.Equal(default, contact);
                continue;
            }

            Assert.Equal(Fixed64.FromRaw(-gapRaw), contact.Depth);
            Assert.Equal(normal, contact.Normal);
            Assert.False(contact.DepthIsClamped);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(side ? Fixed64.One : cone ? -Fixed64.One : Fixed64.One, side ? first.X : first.Y);
            Assert.True(first.X * first.X + first.Z * first.Z <= Fixed64.One);
            Assert.True(first.Y.Abs() <= Fixed64.One);
            Vector3d prismPoint = second - prismOrigin;
            Assert.True(prismPoint.X.Abs() <= Fixed64.One && prismPoint.Y.Abs() <= Fixed64.One
                && prismPoint.Z.Abs() <= Fixed64.One);
            Assert.Equal(-Fixed64.One, Vector3d.Dot(prismPoint, normal));
            Assert.Equal(contact.Depth, Vector3d.Dot(first - second, normal));

            // The reported supporting-axis exit clears the complete solids.
            Vector3d movedCenter = -normal * (contact.Depth + Fixed64.FromRaw(4));
            Assert.False(TryContact(cone, movedCenter, Vector3d.Up, Fixed64.One,
                prismOrigin, Fixed64.Zero, footprint, Fixed64.One, out _));
        }
    }

    [Theory]
    [InlineData(true, 1)]
    [InlineData(false, 1)]
    [InlineData(true, -1)]
    [InlineData(false, -1)]
    public void ObliqueFiniteAxis_IntersectsNearCapFaceWhileClearingOppositeFace(bool cone, int verticalSign)
    {
        Vector3d axis = new Vector3d(3, 4 * verticalSign, 0).Normalized;
        Vector3d center = new(Fixed64.FromFraction(3, 2), Fixed64.Two * verticalSign, Fixed64.Zero);
        // This interior axis point proves overlap independently. The remote
        // prism cap remains beyond both the finite axis and its radial reach.
        Vector3d interior = center - axis * Fixed64.FromFraction(3, 2);
        FixedMathTestHelper.AssertWithinRange(interior.X, -Fixed64.One, Fixed64.One);
        FixedMathTestHelper.AssertWithinRange(interior.Y, -Fixed64.One, Fixed64.One);
        Assert.True(center.Y * verticalSign - Fixed64.Two * axis.Y.Abs() - Fixed64.Quarter > -Fixed64.One);

        bool hit = cone
            ? FixedConvexPrismRelations.TryGetCenteredConeContact(center, FixedQuaternion.Identity, axis,
                (Fixed64)4, Fixed64.Quarter, Vector3d.Zero, Fixed64.Zero, Square, Fixed64.One, out _)
            : FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, FixedQuaternion.Identity, axis,
                (Fixed64)4, Fixed64.Quarter, Vector3d.Zero, Fixed64.Zero, Square, Fixed64.One, out _);
        Assert.True(hit);
    }

    [Theory]
    [InlineData(true, true)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(false, false)]
    public void CurvedContact_AdmitsContainmentInEitherDirectionAndReportsAClearingExit(bool cone, bool curvedInside)
    {
        Fixed64 half = curvedInside ? (Fixed64)4 : Fixed64.FromFraction(1, 8);
        Vector2d[] offsets = { new(-half, -half), new(half, -half), new(half, half), new(-half, half) };
        // The curved solid fits in [-1,1]^3. Conversely the small cube's radial
        // reach sqrt(2)/8 is below the cone's minimum section radius 7/16 there.
        Assert.True(TryContact(cone, Vector3d.Zero, Vector3d.Up, Fixed64.One,
            Vector3d.Zero, Fixed64.Zero, offsets, half, out FixedContactAnchors contact));
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out _));
        Assert.True(contact.SecondAnchor.TryGetPoint(out _));

        // These fixtures are bounded by four units; 32 raw units cover the final
        // rounded normal/translation. This checks clearance, not minimum depth.
        Vector3d movedCenter = -contact.Normal * (contact.Depth + Fixed64.FromRaw(32));
        Vector3d normal = contact.Normal;
        Fixed64 radial = FixedMath.Sqrt(normal.X * normal.X + normal.Z * normal.Z);
        Fixed64 sourceMaximum = Vector3d.Dot(movedCenter, normal) + (cone
            ? FixedMath.Max(normal.Y, radial - normal.Y) : normal.Y.Abs() + radial);
        Fixed64 prismMinimum = -half * (normal.X.Abs() + normal.Y.Abs() + normal.Z.Abs());
        Assert.True(sourceMaximum < prismMinimum);
        Assert.False(TryContact(cone, movedCenter, Vector3d.Up, Fixed64.One,
            Vector3d.Zero, Fixed64.Zero, offsets, half, out _));
    }

    [Theory]
    [InlineData(true, false, 0L)]
    [InlineData(true, false, 1L)]
    [InlineData(true, true, 0L)]
    [InlineData(true, true, 1L)]
    [InlineData(false, false, 0L)]
    [InlineData(false, false, 1L)]
    [InlineData(false, true, 0L)]
    [InlineData(false, true, 1L)]
    public void CurvedContact_CollapsedFootprintsRetainTouchingAndOneRawSeparation(bool cone, bool line, long gapRaw)
    {
        Vector2d[] offsets = line
            ? new[] { new Vector2d(0, -1), Vector2d.Zero, new Vector2d(0, 1) }
            : new[] { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        // Every prism point has X=0. At X=1 the cylinder side, or cone base
        // circle at Y=-1, contains a common point. One extra raw unit removes it.
        bool hit = TryContact(cone, Vector3d.Right * (Fixed64.One + Fixed64.FromRaw(gapRaw)),
            Vector3d.Up, Fixed64.One, Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One, out FixedContactAnchors contact);
        Assert.Equal(gapRaw == 0, hit);
        if (hit)
        {
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.False(contact.DepthIsClamped);
        }
        else
            Assert.Equal(default, contact);
    }

    [Theory]
    [InlineData(true, -1L)]
    [InlineData(true, 0L)]
    [InlineData(true, 1L)]
    [InlineData(false, -1L)]
    [InlineData(false, 0L)]
    [InlineData(false, 1L)]
    public void ZeroRadiusObliqueAxis_RetainsClosedYawedSlabAdmissionAtLargeOrigins(bool cone, long gapRaw)
    {
        Vector3d axis = new Vector3d(3, 4, 0).Normalized;
        Assert.True(axis.IsNormalized());
        Assert.Equal(Fixed64.Zero, axis.Z);
        Fixed64 limit = cone ? Fixed64.MaxValue - (Fixed64)16 : Fixed64.MinValue + (Fixed64)16;
        Vector3d origin = new(limit, limit, limit);
        Vector2d[] rectangle =
        {
            new(-Fixed64.Half, -Fixed64.Quarter), new(Fixed64.Half, -Fixed64.Quarter),
            new(Fixed64.Half, Fixed64.Quarter), new(-Fixed64.Half, Fixed64.Quarter)
        };
        Vector3d center = origin + Vector3d.Forward * (Fixed64.Half + Fixed64.FromRaw(gapRaw));
        // Quarter-turn yaw makes the footprint's world-Z range [-1/2,1/2].
        // The zero-radius segment has constant Z regardless of its rounded XY
        // axis length, and its center is a shared point exactly through this face.
        bool hit = TryContact(cone, center, axis, Fixed64.Zero, origin, Fixed64.HalfPi,
            rectangle, Fixed64.Quarter, out FixedContactAnchors contact);
        Assert.Equal(gapRaw <= 0, hit);
        if (hit)
        {
            Assert.Equal(Fixed64.FromRaw(-gapRaw), contact.Depth);
            Assert.Equal(center, contact.FirstAnchor.Origin);
            Assert.Equal(origin, contact.SecondAnchor.Origin);
            Assert.False(contact.DepthIsClamped);
        }
        else
            Assert.Equal(default, contact);
    }

    [Theory]
    [InlineData(true, -1L)]
    [InlineData(true, 0L)]
    [InlineData(true, 1L)]
    [InlineData(false, -1L)]
    [InlineData(false, 0L)]
    [InlineData(false, 1L)]
    public void NonzeroRadiusObliqueAxis_RetainsClosedEdgeAdmissionWithoutAssumingExactUnitLength(bool cone, long gapRaw)
    {
        Vector3d axis = new Vector3d(3, 4, 0).Normalized;
        Assert.True(axis.IsNormalized());
        Assert.Equal(Fixed64.Zero, axis.Z);
        Assert.True(axis.Y.Abs() < Fixed64.Two);
        Vector2d[] offsets = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Vector3d prismOrigin = new(cone ? -axis.X : Fixed64.Zero,
            Fixed64.Zero, Fixed64.One + Fixed64.FromRaw(gapRaw));
        // Z is exactly perpendicular to the authored axis. The vertical prism
        // contains (0,0,1+gap) on the cylinder's middle disk, or
        // -axis+(0,0,1+gap) on the cone's base disk. Both disks have radius one;
        // a larger Z is strictly beyond the complete primitive's support.
        bool hit = TryContact(cone, Vector3d.Zero, axis, Fixed64.One, prismOrigin,
            Fixed64.Zero, offsets, Fixed64.Two, out FixedContactAnchors contact);
        Assert.Equal(gapRaw <= 0, hit);
        if (!hit)
            Assert.Equal(default, contact);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CoplanarAxisAndPrismFace_RetainExteriorCenterTangency(bool cone)
    {
        // The Z axis lies in the prism's upper plane Y=1, with its center
        // outside X in [-1,1]. The cylinder shares (1,1,0), and the cone's
        // base disk shares (1,1,-1). X=1 separates the interiors exactly.
        Assert.True(TryContact(cone, new Vector3d(2, 1, 0), Vector3d.Forward, Fixed64.One,
            Vector3d.Zero, Fixed64.Zero, Square, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    [InlineData(-1L, true)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    public void ConeContact_ClosesTheBottomEdgeDiskIntersectionWithExteriorVertices(long gapRaw, bool filled)
    {
        Vector2d[] offsets = filled
            ? new[] { new Vector2d(0, -1), new Vector2d(1, -1), new Vector2d(1, 1), new Vector2d(0, 1) }
            : new[] { new Vector2d(0, -1), new Vector2d(0, 1), new Vector2d(0, 1) };
        // The prism has minimum X=1/2+gap, Y in [0,2] and Z in [-1,1].
        // Cone sections at Y>=0 have radius at most 1/2. At gap zero the sole
        // common point is (1/2,0,0), inside the bottom edge; its endpoints,
        // all vertical edges and the cone base at Y=-1 are disjoint.
        bool hit = TryContact(true, Vector3d.Zero, Vector3d.Up, Fixed64.One,
            new Vector3d(Fixed64.Half + Fixed64.FromRaw(gapRaw), Fixed64.One, Fixed64.Zero), Fixed64.Zero,
            offsets, Fixed64.One, out FixedContactAnchors contact);
        Assert.Equal(gapRaw <= 0, hit);
        if (!hit)
            Assert.Equal(default, contact);
    }

    [Theory]
    [InlineData(48, -4, 0, false)]
    [InlineData(48, -44, 40, false)]
    [InlineData(4, -2, 1, true)]
    [InlineData(4, -2, 2, false)]
    public void WideConeEdgeQuadratic_RespectsFiniteAxialClipAndClosedTangency(int a, int b, int c, bool expected)
    {
        const int words = 16;
        Span<ulong> values = stackalloc ulong[6 * words];
        Span<int> signs = stackalloc int[6];
        Span<int> coefficients = stackalloc int[] { a, b, c };
        for (int index = 0; index < coefficients.Length; index++)
        {
            ContactQuadratic coefficient = ContactQuadratic.At(values, signs, index, words);
            coefficient.Clear();
            coefficient.Rational[14] = (ulong)Math.Abs(coefficients[index]);
            coefficient.Signs[0] = Math.Sign(coefficients[index]);
        }

        // Evaluate 2^896 * (a*t^2 + 2*b*t + c) only on [1/4,3/4].
        // The first two stationary points are 1/12 and 11/12: roots outside
        // the finite axial clip cannot admit another cone section. The other
        // rows retain the interior double root at 1/2 and reject a positive gap.
        Signed320 lowerNumerator = Signed320.ExtendValue(Signed192.Signed(1));
        Signed320 denominator = Signed320.ExtendValue(Signed192.Signed(4));
        bool hit = CapsuleFiniteSectionFrame.IntersectsQuadratic(
            ContactQuadratic.At(values, signs, 0, words), ContactQuadratic.At(values, signs, 1, words),
            ContactQuadratic.At(values, signs, 2, words), lowerNumerator, denominator,
            Signed320.ExtendValue(Signed192.Signed(3)), denominator);
        Assert.Equal(expected, hit);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void SeparatedCurvedFixtures_KeepTheirNearbyIndependentlyWitnessedOverlap(bool cone)
    {
        Vector3d center;
        Vector3d origin;
        FixedQuaternion rotation;
        Vector2d[] offsets;
        Fixed64 halfThickness;
        bool hit;
        if (cone)
        {
            center = new Vector3d(Fixed64.FromRaw(-565576967), Fixed64.FromRaw(-557169159), Fixed64.FromRaw(-565576967))
                + Vector3d.One * Fixed64.FromFraction(1, 64);
            Fixed64 localY = -center.Y;
            Fixed64 sectionRadius = Fixed64.Half * (Fixed64.Half - localY);
            // The prism corner (0,0,0) is strictly inside the translated cone.
            Assert.True(localY.Abs() < Fixed64.Half);
            Assert.True(center.X * center.X + center.Z * center.Z < sectionRadius * sectionRadius);
            origin = new Vector3d(2, 1, 2);
            rotation = FixedQuaternion.Identity;
            offsets = new[] { new Vector2d(-2, -2), new Vector2d(2, -2), new Vector2d(2, 2), new Vector2d(-2, 2) };
            halfThickness = Fixed64.One;
            hit = FixedConvexPrismRelations.TryGetCenteredConeContact(center, rotation, Vector3d.Up, Fixed64.One,
                Fixed64.Half, origin, Fixed64.Zero, offsets, halfThickness, out _);
        }
        else
        {
            rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)22, (Fixed64)17, (Fixed64)9);
            Vector3d normal = new(Fixed64.Zero, Fixed64.FromRaw(-3980925575), Fixed64.FromRaw(-1612133878));
            center = new Vector3d(Fixed64.Quarter, Fixed64.FromFraction(3, 4), Fixed64.Quarter)
                - normal * (Fixed64.FromRaw(1918558518) + Fixed64.FromFraction(1, 1024)) - Vector3d.Up * Fixed64.Quarter;
            Vector3d commonPoint = new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Half);
            Vector3d difference = commonPoint - center;
            // This point is on the prism's upper rim and inside the cylinder's
            // inscribed unit ball, independently of its rotation.
            Assert.True(Vector3d.Dot(difference, difference) < Fixed64.One);
            origin = new Vector3d(0, -4, 0);
            offsets = new[] { new Vector2d(-Fixed64.One, -Fixed64.Half), new Vector2d(Fixed64.One, -Fixed64.Half),
                new Vector2d(Fixed64.One, Fixed64.Half), new Vector2d(-Fixed64.One, Fixed64.Half) };
            halfThickness = (Fixed64)4;
            hit = FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up, Fixed64.Two,
                Fixed64.One, origin, Fixed64.Zero, offsets, halfThickness, out _);
        }
        Assert.True(hit);
    }

    [Fact]
    public void CurvedSeparatingFeatures_RemainAllocationFreeAfterWarmup()
    {
        Vector3d coneCenter = new(Fixed64.FromRaw(-565576967), Fixed64.FromRaw(-557169159), Fixed64.FromRaw(-565576967));
        Vector2d[] coneOffsets = { new(-2, -2), new(2, -2), new(2, 2), new(-2, 2) };
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)22, (Fixed64)17, (Fixed64)9);
        Vector3d normal = new(Fixed64.Zero, Fixed64.FromRaw(-3980925575), Fixed64.FromRaw(-1612133878));
        Vector3d cylinderCenter = new Vector3d(Fixed64.Quarter, Fixed64.FromFraction(3, 4), Fixed64.Quarter)
            - normal * (Fixed64.FromRaw(1918558518) + Fixed64.FromFraction(1, 1024));
        Vector2d[] cylinderOffsets = { new(-Fixed64.One, -Fixed64.Half), new(Fixed64.One, -Fixed64.Half),
            new(Fixed64.One, Fixed64.Half), new(-Fixed64.One, Fixed64.Half) };

        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            for (int index = 0; index < 128; index++)
            {
                Assert.False(FixedConvexPrismRelations.TryGetCenteredConeContact(coneCenter, FixedQuaternion.Identity,
                    Vector3d.Up, Fixed64.One, Fixed64.Half, new Vector3d(2, 1, 2), Fixed64.Zero,
                    coneOffsets, Fixed64.One, out _));
                Assert.False(FixedConvexPrismRelations.TryGetCenteredCylinderContact(cylinderCenter, rotation,
                    Vector3d.Up, Fixed64.Two, Fixed64.One, new Vector3d(0, -4, 0), Fixed64.Zero,
                    cylinderOffsets, (Fixed64)4, out _));
            }
        });
        Assert.Equal(0, allocated);
    }

    // Shared fixtures use a two-unit full length and the identity rigid frame.
    private static bool TryContact(bool cone, Vector3d center, Vector3d axis, Fixed64 radius,
        Vector3d prismOrigin, Fixed64 yaw, Vector2d[] offsets, Fixed64 halfThickness, out FixedContactAnchors contact) => cone
        ? FixedConvexPrismRelations.TryGetCenteredConeContact(center, FixedQuaternion.Identity, axis, Fixed64.Two,
            radius, prismOrigin, yaw, offsets, halfThickness, out contact)
        : FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, FixedQuaternion.Identity, axis, Fixed64.Two,
            radius, prismOrigin, yaw, offsets, halfThickness, out contact);
}
