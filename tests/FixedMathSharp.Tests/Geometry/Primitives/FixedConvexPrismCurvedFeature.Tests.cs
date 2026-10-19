using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedConvexPrismCurvedFeatureTests
{
    [Fact]
    public void TinyVerticalPrism_RetainsAnExactNonprincipalRimTouch()
    {
        const long quaternionScale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        Vector2d point = new(Fixed64.FromRaw(-11), Fixed64.FromRaw(20));
        Vector2d[] footprint = { point, point, point };
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(Vector3d.Zero, rotation,
            Vector3d.Up, Fixed64.FromRaw(5), Fixed64.FromRaw(25), Vector3d.Zero, Fixed64.Zero,
            footprint, Fixed64.FromRaw(30), out FixedContactAnchors contact));
        // The exact rim point is(-11,10.5,20) raw, with supporting normal
        // proportional to(-5,0,4). The complete vertical target segment has
        // the same support value and includes that point, proving tangency.
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.Normal.X < Fixed64.Zero && contact.Normal.Z > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.InRange(4 * contact.Normal.X.m_rawValue + 5 * contact.Normal.Z.m_rawValue, -5L, 5L);
        Vector3d roundedLocal = new(Fixed64.FromRaw(-15), Fixed64.FromRaw(-2), Fixed64.FromRaw(20));
        Assert.Equal(roundedLocal, contact.FirstAnchor.LocalPoint);
        Assert.Equal(roundedLocal, contact.SecondAnchor.LocalPoint);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalDisplacement);
        Assert.Equal(Vector3d.Zero, contact.SecondAnchor.LocalDisplacement);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(first, second);
    }

    [Fact]
    public void ObliqueRealEdge_PrincipalDirectionMatchesTheCompleteBoxMinimum()
    {
        const long quaternionScale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        FixedQuaternion boxRotation = new(Fixed64.Zero, Fixed64.FromRaw(-quaternionScale),
            Fixed64.Zero, Fixed64.FromRaw(2 * quaternionScale));
        Vector3d edgePoint = new(Fixed64.FromFraction(15, 4), Fixed64.FromFraction(693, 256),
            Fixed64.FromFraction(3, 16));
        Vector3d origin = edgePoint + Vector3d.Up * (Fixed64)5;
        Fixed64 widthX = Fixed64.FromFraction(1, 16), widthZ = Fixed64.FromFraction(3, 64);
        Vector2d[] footprint = { new(-3, -4), new(3, 4),
            new((Fixed64)3 + widthX, (Fixed64)4 - widthZ), new((Fixed64)(-3) + widthX, (Fixed64)(-4) - widthZ) };
        var box = new FixedOrientedBox(origin + new Vector3d(widthX / Fixed64.Two, Fixed64.Zero, -widthZ / Fixed64.Two),
            boxRotation, new Vector3d((Fixed64)5, (Fixed64)5, Fixed64.FromFraction(5, 128)));
        Assert.True(box.TryGetCenteredCylinderContact(Vector3d.Zero, rotation, Vector3d.Up,
            (Fixed64)10, Fixed64.One, out FixedContactAnchors reference));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(Vector3d.Zero, rotation,
            Vector3d.Up, (Fixed64)10, Fixed64.One, origin, Fixed64.Zero, footprint, (Fixed64)5,
            out FixedContactAnchors contact));
        // With A=(4,3,0)/5,E=(3,0,4), N=(64,75,-48) is perpendicular
        // to E and parallel to the projected axis. The cap center H=(4,3,0)
        // minus the edge point is N/256, so both normalized linear and disk
        // derivatives vanish at this principal direction. The finite strip
        // is exactly the rational-yaw box above; no rounded yaw is involved.
        Assert.Equal(reference.Depth, contact.Depth);
        Assert.Equal(-reference.Normal, contact.Normal);
        Assert.InRange(contact.Depth.m_rawValue, 1L, Fixed64.One.m_rawValue - 1);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Vector3d relative = second - edgePoint;
        long alongEdge = 3 * relative.X.m_rawValue + 4 * relative.Z.m_rawValue;
        long acrossEdge = 4 * relative.X.m_rawValue - 3 * relative.Z.m_rawValue;
        Assert.InRange(alongEdge, -25L * (1L << 32) - 14, 25L * (1L << 32) + 14);
        Assert.InRange(acrossEdge, -14L, 25L * Fixed64.FromFraction(1, 64).m_rawValue + 14);
        Assert.InRange(second.Y.m_rawValue, edgePoint.Y.m_rawValue - 2,
            (edgePoint.Y + (Fixed64)10).m_rawValue + 2);
        AssertPairedWithinRaw(first, second, contact, 8);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            -contact.Normal * (contact.Depth + Fixed64.FromRaw(32)), rotation, Vector3d.Up,
            (Fixed64)10, Fixed64.One, origin, Fixed64.Zero, footprint, (Fixed64)5, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NearEllipseCaustic_RetainsTheMinimumAcrossCompetingStationaryRoots(bool nearCusp)
    {
        const long quaternionScale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        Fixed64 narrow = Fixed64.FromRaw(1);
        Fixed64 majorCoordinate = nearCusp
            ? (Fixed64)16 - Fixed64.FromFraction(1, 262144)
            : Fixed64.FromFraction(31, 2);
        Vector3d target = new((Fixed64)(-4) + narrow, Fixed64.Zero, majorCoordinate);
        Vector2d[] footprint = { new(-narrow, -narrow), new(narrow, -narrow),
            new(narrow, narrow), new(-narrow, narrow) };
        var box = new FixedOrientedBox(target, FixedQuaternion.Identity,
            new Vector3d(narrow, (Fixed64)30, narrow));
        Assert.True(box.TryGetCenteredCylinderContact(Vector3d.Zero, rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, out FixedContactAnchors reference));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(Vector3d.Zero, rotation,
            Vector3d.Up, (Fixed64)10, (Fixed64)25, target, Fixed64.Zero, footprint, (Fixed64)30,
            out FixedContactAnchors contact));
        // The bottom cap projects to an ellipse with radii15 and25. The
        // Both targets lie just off its major axis below the caustic16;
        // the second approaches the cusp while retaining distinct extrema.
        // Nearby stationary maxima and minima must not be confused when
        // selecting the complete minimum. The independent box owner has
        // exactly the same finite region, including its one-raw widths.
        Assert.Equal(reference.Depth, contact.Depth);
        Assert.Equal(-reference.Normal, contact.Normal);
        Assert.InRange(contact.Depth.m_rawValue, Fixed64.One.m_rawValue, ((Fixed64)10).m_rawValue);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.InRange(second.X.m_rawValue - target.X.m_rawValue, -3L, 3L);
        Assert.InRange(second.Z.m_rawValue - target.Z.m_rawValue, -3L, 3L);
        Assert.InRange(second.Y.m_rawValue, -30L * (1L << 32) - 2, 30L * (1L << 32) + 2);
        AssertPairedWithinRaw(first, second, contact, 16);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            -contact.Normal * (contact.Depth + Fixed64.FromRaw(64)), rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, target, Fixed64.Zero, footprint, (Fixed64)30, out _));
    }

    [Theory]
    [InlineData(5L, -2L)]
    [InlineData(15L, -8L)]
    public void TinyFiniteRim_RoundsExactCapHalfTiesToEven(long lengthRaw, long expectedLocalY)
    {
        const long quaternionScale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        Vector3d origin = new(Fixed64.FromRaw(-2 * lengthRaw / 5 - 4), Fixed64.FromRaw(5), Fixed64.FromRaw(16));
        Vector2d[] footprint = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(Vector3d.Zero, rotation,
            Vector3d.Up, Fixed64.FromRaw(lengthRaw), Fixed64.FromRaw(25), origin, Fixed64.Zero,
            footprint, Fixed64.FromRaw(25), out FixedContactAnchors contact));
        // In raw units, the cap disk projects to the ellipse with radii15
        // and25. Its point S=(-9,20) has normal(-5,4), and the target
        // projection is P=(-4,16). The radius-sqrt(41) disk about P fits
        // the ellipse: with G=diag(1/225,1/625), |GS|=sqrt(41)/125
        // exceeds sqrt(41)*lambdaMax(G). Lifting a ball vector v requires
        // targetY=16/3-3L/10-4vX/3-vY; |v|=sqrt(41)<7 puts targetY
        // inside[-11,16], within the authored[-20,30] segment. The matching
        // supporting plane proves the exact minimum sqrt(41) raw, rounding6.
        Assert.Equal(Fixed64.FromRaw(6), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.Normal.X < Fixed64.Zero && contact.Normal.Z > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.InRange(4 * contact.Normal.X.m_rawValue + 5 * contact.Normal.Z.m_rawValue, -5L, 5L);
        // The exact primitive-frame point is(-15,-L/2,20) raw. These two
        // half ties exercise an odd and an even floor before any world-frame
        // rounding, which need not commute with local coordinate rounding.
        Assert.Equal(new Vector3d(Fixed64.FromRaw(-15), Fixed64.FromRaw(expectedLocalY), Fixed64.FromRaw(20)),
            contact.FirstAnchor.LocalPoint);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalDisplacement);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.InRange(second.X.m_rawValue - origin.X.m_rawValue, -2L, 2L);
        Assert.InRange(second.Z.m_rawValue - origin.Z.m_rawValue, -2L, 2L);
        Assert.InRange(second.Y.m_rawValue, -22L, 32L);
        AssertPairedWithinRaw(first, second, contact, 8);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void RebasedThinPrism_MatchesTheCompleteBoxRimMinimumAndPairedFeatures(int mirror)
    {
        const long scaleRaw = 1L << 58;
        Fixed64 scale = Fixed64.FromRaw(scaleRaw);
        Fixed64 narrow = Fixed64.FromFraction(1, 64);
        Vector3d target = new((Fixed64)mirror * scale / (Fixed64)64, Fixed64.Zero, scale * (Fixed64)4);
        Vector3d prismOrigin = new((Fixed64)mirror * (Fixed64.MaxValue - Fixed64.One), Fixed64.Zero, target.Z);
        Fixed64 localX = target.X - prismOrigin.X;
        Vector2d[] footprint = { new(localX - narrow, -narrow), new(localX + narrow, -narrow),
            new(localX + narrow, narrow), new(localX - narrow, narrow) };
        const long quaternionScale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        Fixed64 length = scale * (Fixed64)10, radius = scale * (Fixed64)25, halfY = scale * (Fixed64)30;
        var box = new FixedOrientedBox(target, FixedQuaternion.Identity, new Vector3d(narrow, halfY, narrow));
        Assert.True(box.TryGetCenteredCylinderContact(Vector3d.Zero, rotation, Vector3d.Up,
            length, radius, out FixedContactAnchors reference));
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(Vector3d.Zero, rotation,
            Vector3d.Up, length, radius, prismOrigin, Fixed64.Zero, footprint, halfY,
            out FixedContactAnchors contact));

        // The target contains its center p, with |p|<4.1F. The cylinder
        // contains ball(0,5F), proving a positive exit greater than.9F.
        // Its X support gives an upper bound below19F. Side normals have
        // gap>20.99F and cap poles have gap near23F, so the winner is a rim.
        // The equivalent complete box solver supplies a separate minimum
        // oracle; its support anchors do not define these paired witnesses.
        Assert.Equal(reference.Depth, contact.Depth);
        Assert.Equal(-reference.Normal, contact.Normal);
        Assert.False(contact.DepthIsClamped);
        Assert.True(10 * (BigInteger)contact.Depth.m_rawValue > 9 * (BigInteger)scaleRaw);
        Assert.True(contact.Depth.m_rawValue < 19 * scaleRaw);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(prismOrigin, contact.SecondAnchor.Origin);
        Assert.InRange(second.X.m_rawValue - target.X.m_rawValue, -narrow.m_rawValue - 2, narrow.m_rawValue + 2);
        Assert.InRange(second.Z.m_rawValue - target.Z.m_rawValue, -narrow.m_rawValue - 2, narrow.m_rawValue + 2);
        Assert.InRange(second.Y.m_rawValue, -halfY.m_rawValue - 2, halfY.m_rawValue + 2);

        // Exact world axis(4,3,0)/5 gives cap projection +/-5F and
        // rim norm²=(5F)²+(25F)². Allow only final-frame coordinate rounding.
        BigInteger cap = 4 * (BigInteger)first.X.m_rawValue + 3 * (BigInteger)first.Y.m_rawValue;
        Assert.True(BigInteger.Abs(BigInteger.Abs(cap) - 25 * (BigInteger)scaleRaw) <= 20);
        BigInteger squared = BigInteger.Pow(first.X.m_rawValue, 2) + BigInteger.Pow(first.Y.m_rawValue, 2)
            + BigInteger.Pow(first.Z.m_rawValue, 2);
        Assert.True(BigInteger.Abs(squared - 650 * BigInteger.Pow(scaleRaw, 2)) <= 512 * (BigInteger)scaleRaw);
        Vector3d roundedDisplacement = contact.Normal * contact.Depth;
        long pairingTolerance = contact.Depth.m_rawValue / (2L << 32) + 8;
        for (int component = 0; component < 3; component++)
            Assert.True(BigInteger.Abs((BigInteger)first[component].m_rawValue - second[component].m_rawValue
                - roundedDisplacement[component].m_rawValue) <= pairingTolerance);
        // At this depth, normal-component rounding can move an exit by a
        // fraction of a physical unit. Two units cover that error; translate
        // the primitive to keep the distant authored prism origin in range.
        Vector3d clearedCenter = -contact.Normal * (contact.Depth + Fixed64.Two);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCylinderContact(clearedCenter, rotation,
            Vector3d.Up, length, radius, prismOrigin, Fixed64.Zero, footprint, halfY, out _));
    }

    [Fact]
    public void FullDomainContainment_ClampsTheTrueMinimumAndRetainsScaledPhysicalWitnesses()
    {
        Fixed64 extent = Fixed64.MaxValue;
        Vector2d[] footprint = { new(-extent, -extent), new(extent, -extent),
            new(extent, extent), new(-extent, extent) };
        FixedQuaternion rotation = new FixedQuaternion((Fixed64)3, (Fixed64)(-2), (Fixed64)5, (Fixed64)7).Normalized;
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(Vector3d.Zero, rotation,
            Vector3d.Up, extent, extent, Vector3d.Zero, Fixed64.Zero, footprint, extent,
            out FixedContactAnchors contact));
        // The cube contains ball(0,Max), while the cylinder contains a ball
        // larger than Max/3. Their Minkowski sum therefore contains a ball
        // larger than Max, independently proving that the minimum clamps.
        Assert.True(contact.DepthIsClamped);
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.Normal.IsNormalized());
        var origin = new FixedPointAnchor(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero);
        Assert.True(contact.FirstAnchor.TryGetScaledOffsetFrom(origin, Fixed64.Quarter, out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetScaledOffsetFrom(origin, Fixed64.Quarter, out Vector3d second));
        BigInteger projection = 0;
        for (int component = 0; component < 3; component++)
            projection += ((BigInteger)first[component].m_rawValue - second[component].m_rawValue)
                * contact.Normal[component].m_rawValue;
        Assert.True(projection > ((BigInteger)long.MaxValue / 4) * (1L << 32));
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.Origin);
        Assert.Equal(Vector3d.Zero, contact.SecondAnchor.Origin);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ConeAxisVertex_RetainsTheFiniteGeneratorMinimum(int axisIndex)
    {
        Vector3d axis = axisIndex == 0 ? Vector3d.Right : axisIndex == 1 ? Vector3d.Up : Vector3d.Forward;
        Vector2d[] footprint = { new(0, 0), new(1, 0), new(0, 1) };
        Vector3d origin = Vector3d.Up;
        Assert.True(FixedConvexPrismRelations.TryGetCenteredConeContact(Vector3d.Zero,
            FixedQuaternion.Identity, axis, Fixed64.Two, Fixed64.One, origin, Fixed64.Zero,
            footprint, Fixed64.One, out FixedContactAnchors contact));

        // Height2,radius1 contains the origin-centered ball of radius 1/sqrt(5).
        // The prism contains its origin-corner world point0. A positive
        // generator normal has prism minimum0 and cone support1/sqrt(5),
        // proving the matching upper and lower bounds for every axis here.
        Assert.Equal(Fixed64.FromRaw(1_920_767_767L), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        // Every positive-octant generator normal is an equal minimum here;
        // an earlier rational feature may retain a different azimuth.
        Assert.Equal(Fixed64.FromRaw(1_920_767_767L), Vector3d.Dot(contact.Normal, axis));
        Assert.True(contact.Normal.IsNormalized());
        Assert.True(contact.Normal.X >= Fixed64.Zero && contact.Normal.Y >= Fixed64.Zero
            && contact.Normal.Z >= Fixed64.Zero);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Fixed64 fifth = Fixed64.FromFraction(1, 5);
        Assert.Equal(fifth, Vector3d.Dot(first, axis));
        Assert.InRange(first.MagnitudeSquared.m_rawValue, fifth.m_rawValue - 4, fifth.m_rawValue + 4);
        Assert.Equal(Vector3d.Zero, second);
        Vector3d displacement = contact.Normal * contact.Depth;
        for (int component = 0; component < 3; component++)
            Assert.InRange((first[component] - displacement[component]).m_rawValue, -2L, 2L);
        Vector3d cleared = origin + contact.Normal * (contact.Depth + Fixed64.FromRaw(8));
        Assert.False(FixedConvexPrismRelations.TryGetCenteredConeContact(Vector3d.Zero,
            FixedQuaternion.Identity, axis, Fixed64.Two, Fixed64.One, cleared, Fixed64.Zero,
            footprint, Fixed64.One, out _));
    }

    [Theory]
    [InlineData(4)]
    [InlineData(5)]
    public void RotatedCone_VerticalEdgeRetainsGeneratorTangencyAndTheBaseRim(int radius)
    {
        const long quaternionScale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        Fixed64 delta = Fixed64.FromFraction(1, 64);
        Fixed64 supportX = Fixed64.FromFraction(radius == 4 ? 6 : 9, 5);
        Fixed64 supportY = Fixed64.FromFraction(radius == 4 ? -8 : -49, radius == 4 ? 5 : 10);
        Vector3d origin = new(supportX - delta, supportY, Fixed64.Zero);
        Vector2d[] footprint = { new(0, -1), new(1, -1), new(1, 1), new(0, 1) };
        Assert.True(FixedConvexPrismRelations.TryGetCenteredConeContact(Vector3d.Zero,
            rotation, Vector3d.Up, (Fixed64)3, (Fixed64)radius, origin, Fixed64.Zero,
            footprint, Fixed64.Half, out FixedContactAnchors contact));

        // The exact rotated axis is (4,3,0)/5. For R4, the cone's
        // rightmost generator is vertical at x6/5 and includes y-8/5.
        // For R5, the rightmost base-rim point is (9/5,-49/10,0).
        // In both rows the prism contains a ball centered at that support
        // with radius its +X gap. The matching plane proves the global
        // minimum; fractional support rounding changes it by <half a raw.
        // The real vertical edge has L²*(1-aY²)-R²*aY² equal0 or negative.
        Assert.Equal(delta, contact.Depth);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        // Each retained local coordinate rounds before this nontrivial
        // quaternion frame is materialized, so compare world coordinates
        // within two raw units rather than assuming those roundings commute.
        Assert.InRange(first.X.m_rawValue - supportX.m_rawValue, -2L, 2L);
        Assert.InRange(second.X.m_rawValue - origin.X.m_rawValue, -2L, 2L);
        Assert.InRange(first.Y.m_rawValue - second.Y.m_rawValue, -2L, 2L);
        Assert.InRange(first.Z.m_rawValue, -2L, 2L);
        Assert.InRange(second.Z.m_rawValue, -2L, 2L);
        Assert.InRange(first.Y.m_rawValue, (supportY - Fixed64.Half).m_rawValue - 2,
            (supportY + Fixed64.Half).m_rawValue + 2);
        if (radius == 5)
            Assert.InRange(first.Y.m_rawValue - supportY.m_rawValue, -2L, 2L);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredConeContact(Vector3d.Zero,
            rotation, Vector3d.Up, (Fixed64)3, (Fixed64)radius,
            origin + Vector3d.Right * (contact.Depth + Fixed64.FromRaw(4)), Fixed64.Zero,
            footprint, Fixed64.Half, out _));
    }

    private static void AssertPairedWithinRaw(Vector3d first, Vector3d second, FixedContactAnchors contact, long tolerance)
    {
        Vector3d displacement = contact.Normal * contact.Depth;
        for (int component = 0; component < 3; component++)
            Assert.True(BigInteger.Abs((BigInteger)first[component].m_rawValue - second[component].m_rawValue
                - displacement[component].m_rawValue) <= tolerance);
    }
}
