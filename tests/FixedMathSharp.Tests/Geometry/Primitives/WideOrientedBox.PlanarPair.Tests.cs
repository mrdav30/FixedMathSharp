using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class WideOrientedBoxPlanarPairTests
{
    private static readonly int[] QuadTriangles = { 0, 1, 2, 0, 2, 3 }, QuadEdges = { 0, 1, 1, 2, 2, 3, 3, 0 };
    private static readonly int[] Triangle = { 0, 1, 2 }, TriangleEdges = { 0, 1, 1, 2, 2, 0 };

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoplanarQuadsInIndependentFrames_ShouldRetainTheirOwnedInPlaneSeparators(bool tilted)
    {
        Vector3d[] quad = { new(-1, 0, -1), new(1, 0, -1), new(1, 0, 1), new(-1, 0, 1) };
        FixedQuaternion first = tilted
            ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Two).Normalized
            : FixedQuaternion.Identity;
        // This exact component permutation rotates 180 degrees in the first
        // sheet, preserving its normal and norm even for a tilted plane.
        FixedQuaternion second = new(Fixed64.Zero, first.W, first.X, Fixed64.Zero);
        foreach (bool reverse in new[] { false, true })
        {
            Vector3d firstOrigin = reverse ? Vector3d.Right * (Fixed64)3 : Vector3d.Zero;
            Vector3d secondOrigin = reverse ? Vector3d.Zero : Vector3d.Right * (Fixed64)3;
            Assert.False(WideOrientedBox.TryGetConvexHullPenetration(firstOrigin, reverse ? second : first,
                quad, QuadTriangles, QuadEdges, secondOrigin, reverse ? first : second,
                quad, QuadTriangles, QuadEdges, out var separated, skipParallelDirections: true, planarPair: true));
            Assert.False(separated.HasValue);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void PlanarTrianglePairs_ShouldMatchNativeFiniteTriangleSeparation(int placement)
    {
        Vector3d[] first = { new(0, 0, 0), new(4, 0, 0), new(0, 4, 0) };
        Vector3d[] second = placement == 0 ? new Vector3d[] { new(1, 1, 0), new(3, 1, 0), new(1, 3, 0) }
            : placement == 1 ? new Vector3d[] { new(3, 3, 0), new(5, 3, 0), new(3, 5, 0) }
            : new Vector3d[] { new(3, -2, 0), new(5, 0, 0), new(5, -1, 0) };
        foreach (bool reverse in new[] { false, true })
        {
            var a = reverse ? second : first; var b = reverse ? first : second;
            Assert.Equal(placement == 0, WideTriangleRelations.TryGetPenetration(new(a[0], a[1], a[2]),
                Vector3d.Zero, FixedQuaternion.Identity, new(b[0], b[1], b[2]), Vector3d.Zero, FixedQuaternion.Identity,
                out _, Reject));
            Assert.Equal(placement == 0, WideOrientedBox.TryGetConvexHullPenetration(Vector3d.Zero, FixedQuaternion.Identity,
                a, Triangle, TriangleEdges, Vector3d.Zero, FixedQuaternion.Identity, b, Triangle, TriangleEdges,
                out var result, admission: Reject, planarPair: true));
            Assert.False(result.HasValue);
        }

        static bool Reject(in WidePointSpanPenetration _) => false;
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void PlanarSignedAdmission_ShouldRankTheActualAllowedExitAndKeepFiniteWitnesses(int sign)
    {
        Vector3d[] moving = { new(Fixed64.One, Fixed64.One, -Fixed64.Quarter),
            new Vector3d((Fixed64)3, Fixed64.One, -Fixed64.Quarter), new(3, 1, 1), new(1, 1, 1) };
        Vector3d[] sheet = { new(0, 0, 0), new(4, 0, 0), new(4, 4, 0), new(0, 4, 0) };
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(Vector3d.Zero, FixedQuaternion.Identity,
            moving, QuadTriangles, QuadEdges, Vector3d.Zero, FixedQuaternion.Identity, sheet, QuadTriangles, QuadEdges,
            out var result, admission: Admit, planarPair: true));
        Assert.True(result.HasValue);
        WideAxis3 direction = result.Negate ? -result.Axis : result.Axis;
        Assert.True(direction.X.IsZero && direction.Y.IsZero); Assert.Equal(sign, direction.Z.Sign);
        Assert.Equal(sign > 0 ? Fixed64.One : Fixed64.Quarter, result.GetRoundedDepth(out bool clamped));
        Assert.False(clamped);
        int count = WideOrientedBox.VisitConvexHullContacts(Vector3d.Zero, FixedQuaternion.Identity,
            moving, QuadTriangles, QuadEdges, Vector3d.Zero, FixedQuaternion.Identity, sheet, QuadTriangles, QuadEdges,
            result, sign, static (expectedSign, contact) =>
            {
                Assert.True(contact.FirstAnchor.TryGetPoint(out var first));
                Assert.True(contact.SecondAnchor.TryGetPoint(out var second));
                Assert.Equal(expectedSign > 0 ? Fixed64.One : -Fixed64.Quarter, first.Z);
                Assert.Equal(Fixed64.Zero, second.Z);
                Assert.Equal(Fixed64.One, second.Y);
                Assert.InRange(second.X.m_rawValue, Fixed64.One.m_rawValue, ((Fixed64)3).m_rawValue);
            });
        Assert.True(count > 0);

        bool Admit(in WidePointSpanPenetration candidate)
        {
            WideAxis3 axis = candidate.Negate ? -candidate.Axis : candidate.Axis;
            return axis.X.IsZero && axis.Y.IsZero && axis.Z.Sign == sign;
        }
    }
}
