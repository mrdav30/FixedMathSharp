using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredFinitePairsObliqueFrameStrictOverlapTests
{
    // Keep these integer ratios exact; calling Normalized again could change
    // a component's last raw bit and lose the exact composed-frame identity.
    private static readonly FixedQuaternion CommonFrame = new(
        Fixed64.FromRaw(1_431_655_765L), Fixed64.FromRaw(2_863_311_530L),
        Fixed64.FromRaw(2_863_311_530L), Fixed64.Zero);
    private static readonly FixedQuaternion PerpendicularFrame = new(
        Fixed64.FromRaw(-1_012_333_500L), Fixed64.FromRaw(3_037_000_500L),
        Fixed64.FromRaw(2_024_667_000L), Fixed64.FromRaw(2_024_667_000L));
    private static readonly Vector3d Translation = new(17, -29, 43);

    [Fact]
    public void OracleFrames_AreAdmittedDistinctNormalizedRotations()
    {
        Assert.True(CommonFrame.IsNormalized());
        Assert.True(PerpendicularFrame.IsNormalized());
        Assert.NotEqual(CommonFrame, PerpendicularFrame);
        // Ratios (1,2,2,0) and (-1,3,2,2) compose exactly as
        // q_common * (0,0,-1,1), regardless of their different raw scales.
        // Their local +Y axes are respectively (4,-1,8)/9 and (-7,4,4)/9.
    }

    [Theory]
    [InlineData(-1, false, -1L, true)]
    [InlineData(-1, false, 0L, false)]
    [InlineData(-1, false, 1L, false)]
    [InlineData(-1, true, -1L, true)]
    [InlineData(-1, true, 0L, false)]
    [InlineData(-1, true, 1L, false)]
    [InlineData(1, false, -1L, true)]
    [InlineData(1, false, 0L, false)]
    [InlineData(1, false, 1L, false)]
    [InlineData(1, true, -1L, true)]
    [InlineData(1, true, 0L, false)]
    [InlineData(1, true, 1L, false)]
    public void Cylinders_PreserveExactObliqueRimBoundary(int mirror, bool swap, long rawOffset, bool expected)
    {
        Vector3d baselineCenter = new((Fixed64)7 / 4, (Fixed64)7 / 4,
            mirror * (Fixed64.Two + Fixed64.FromRaw(rawOffset)));
        // At x/y offsets 3/4, each radius 5/4 disk has Z reach 1. Their
        // combined reach 2 equals the center's absolute Z at exact tangency.
        Assert.Equal(expected, Cylinders(baselineCenter, (Fixed64)5 / 4, (Fixed64)5 / 4, swap));
    }

    [Theory]
    [InlineData(-1, -1L, false)]
    [InlineData(-1, 0L, false)]
    [InlineData(-1, 1L, true)]
    [InlineData(1, -1L, false)]
    [InlineData(1, 0L, false)]
    [InlineData(1, 1L, true)]
    public void CylinderCone_PreserveExactObliqueGeneratorBoundary(int mirror, long rawOffset, bool expected)
    {
        // In the baseline meridian the cone H4/R3 has outward unit normal
        // (0,3/5,+/-4/5). The cylinder center is 5 units from the generator
        // midpoint (0,0,+/-3/2), so radius 5 is exactly tangent.
        Vector3d baselineCenter = new(Fixed64.Zero, (Fixed64)3, mirror * (Fixed64)11 / 2);
        Assert.Equal(expected, CylinderCone(baselineCenter, (Fixed64)5 + Fixed64.FromRaw(rawOffset)));
    }

    [Fact]
    public void ObliqueCylinders_MatchIndependentDiskIntervalOracle()
    {
        for (int sample = 0; sample < 128; sample++)
        {
            Fixed64 firstRadius = Fixed64.One + (Fixed64)(sample % 3) / 4;
            Fixed64 secondRadius = Fixed64.One + (Fixed64)(sample / 3 % 3) / 4;
            Vector3d center = new((Fixed64)(sample * 13 % 65 - 32) / 8,
                (Fixed64)(sample * 19 % 65 - 32) / 8, (Fixed64)(sample * 29 % 65 - 32) / 8);
            // Eliminate the baseline X/Y intervals analytically. Strict
            // overlap requires positive squared Z reaches A,B and |z|<sqrt(A)+sqrt(B).
            BigInteger x = BigInteger.Max(BigInteger.Abs(center.X.m_rawValue) - Fixed64.One.m_rawValue, 0);
            BigInteger y = BigInteger.Max(BigInteger.Abs(center.Y.m_rawValue) - Fixed64.One.m_rawValue, 0);
            BigInteger a = Square(firstRadius.m_rawValue) - Square(x);
            BigInteger b = Square(secondRadius.m_rawValue) - Square(y);
            BigInteger difference = Square(center.Z.m_rawValue) - a - b;
            bool expected = a > 0 && b > 0 && (difference < 0 || Square(difference) < 4 * a * b);

            bool actual = Cylinders(center, firstRadius, secondRadius, swap: false);
            bool swapped = Cylinders(center, firstRadius, secondRadius, swap: true);
            Assert.True(actual == expected && swapped == expected,
                $"Oblique cylinder sample {sample}: expected {expected}, actual {actual}, swapped {swapped}.");
        }
    }

    [Fact]
    public void ObliqueCylinderCone_MatchesIndependentMeridianTriangleOracle()
    {
        for (int sample = 0; sample < 128; sample++)
        {
            Fixed64 y = (Fixed64)(sample * 17 % 81 - 40) / 8;
            Fixed64 z = (Fixed64)(sample * 11 % 81 - 40) / 8;
            Fixed64 radius = Fixed64.Half + (Fixed64)(sample % 5) / 4;
            // Before the exact common rotation, both shapes are closest in
            // the x=0 plane. Thus this is precisely a disk against the cone's
            // YZ meridian triangle, not a sampled 3D collision oracle.
            long halfHeight = Fixed64.Two.m_rawValue;
            long coneRadius = ((Fixed64)3).m_rawValue;
            bool expected = CircleEntersTriangle(y.m_rawValue, z.m_rawValue, radius.m_rawValue,
                -halfHeight, -coneRadius, -halfHeight, coneRadius, halfHeight, 0);
            bool actual = CylinderCone(new Vector3d(Fixed64.Zero, y, z), radius);
            Assert.True(actual == expected,
                $"Oblique meridian sample {sample}: expected {expected}, actual {actual}.");
        }
    }

    [Fact]
    public void ParallelObliqueCylinders_MatchFullDomainIntegerOracle()
    {
        const long maximum = long.MaxValue;
        const long minimum = long.MinValue;
        (Vector3d First, Vector3d Second, long FirstHeight, long SecondHeight,
            long FirstRadius, long SecondRadius)[] scenes =
        {
            (RawVector(minimum, minimum, minimum), RawVector(maximum, maximum, maximum),
                maximum, maximum - 1, maximum, maximum - 1),
            (RawVector(minimum, 0, 0), RawVector(maximum, 0, 0),
                maximum, maximum - 1, maximum, maximum - 1),
            (RawVector(0, minimum, 0), RawVector(0, maximum, 0),
                maximum - 1, maximum, maximum - 2, maximum),
            (RawVector(0, 0, minimum), RawVector(0, 0, maximum),
                maximum, maximum - 1, maximum, maximum),
            (RawVector(minimum, 0, 0), RawVector(maximum, 0, 0),
                maximum, maximum, maximum / 2, maximum / 2),
            (RawVector(minimum, 0, 0), RawVector(maximum, 0, 0),
                maximum / 8 * 7, maximum / 8 * 7, maximum, maximum),
            (RawVector(minimum, maximum, minimum), RawVector(minimum, maximum, minimum),
                maximum - 2, maximum - 3, maximum - 4, maximum - 5),
            (RawVector(minimum, maximum, minimum), RawVector(maximum, minimum, maximum),
                maximum, maximum, maximum, maximum)
        };

        for (int index = 0; index < scenes.Length; index++)
        {
            var scene = scenes[index];
            bool expected = ParallelCylinderOracle(scene.First, scene.Second,
                scene.FirstHeight, scene.SecondHeight, scene.FirstRadius, scene.SecondRadius);
            bool actual = ParallelCylinders(scene.First, scene.Second,
                scene.FirstHeight, scene.SecondHeight, scene.FirstRadius, scene.SecondRadius);
            bool swapped = ParallelCylinders(scene.Second, scene.First,
                scene.SecondHeight, scene.FirstHeight, scene.SecondRadius, scene.FirstRadius);
            Assert.True(actual == expected && swapped == expected,
                $"Full-domain parallel scene {index}: expected {expected}, actual {actual}, swapped {swapped}.");
        }
    }

    [Theory]
    [InlineData(false, -1L)]
    [InlineData(false, 0L)]
    [InlineData(false, 1L)]
    [InlineData(true, -1L)]
    [InlineData(true, 0L)]
    [InlineData(true, 1L)]
    public void ParallelObliqueCylinders_PreserveNearMaximumExactBoundaries(bool axial, long rawOffset)
    {
        const long unit = long.MaxValue / 9;
        Vector3d second = axial
            ? RawVector(4 * unit, -unit, 8 * unit)
            : RawVector(-7 * unit, 4 * unit, 4 * unit);
        long firstHeight = axial ? 9 * unit : long.MaxValue;
        long secondHeight = axial ? 9 * unit + rawOffset : long.MaxValue;
        long firstRadius = axial ? long.MaxValue : 4 * unit;
        long secondRadius = axial ? long.MaxValue : 5 * unit + rawOffset;
        // The integer triples (4,-1,8) and (-7,4,4) have length 9 and are
        // orthogonal. Thus the axial or radial separation is exactly 9*unit;
        // only the one-raw dimension change selects penetration or clearance.
        bool expected = rawOffset > 0;
        Assert.Equal(expected, ParallelCylinderOracle(Vector3d.Zero, second,
            firstHeight, secondHeight, firstRadius, secondRadius));
        Assert.Equal(expected, ParallelCylinders(Vector3d.Zero, second,
            firstHeight, secondHeight, firstRadius, secondRadius));
    }

    [Fact]
    public void DistinctObliqueFrames_SeparateMaximumSolidsAcrossOppositeExtremeCenters()
    {
        const long firstFactor = 460_468_827L;
        const long secondFactor = 687_745_183L;
        FixedQuaternion firstRotation = new(Fixed64.FromRaw(2 * firstFactor),
            Fixed64.FromRaw(3 * firstFactor), Fixed64.FromRaw(5 * firstFactor), Fixed64.FromRaw(7 * firstFactor));
        FixedQuaternion secondRotation = new(Fixed64.FromRaw(secondFactor),
            Fixed64.FromRaw(5 * secondFactor), Fixed64.FromRaw(-3 * secondFactor), Fixed64.FromRaw(2 * secondFactor));
        Assert.True(firstRotation.IsNormalized());
        Assert.True(secondRotation.IsNormalized());

        Vector3d first = RawVector(long.MinValue, long.MinValue, long.MinValue);
        Vector3d second = RawVector(long.MaxValue, long.MaxValue, long.MaxValue);
        Fixed64 size = Fixed64.MaxValue;
        // Either centered H=M/R=M solid fits within a sphere of radius
        // sqrt(5)*M/2, independent of its orientation. The exact squared
        // center distance exceeds the squared sum 5*M^2 of those bounds.
        BigInteger delta = (BigInteger)long.MaxValue - long.MinValue;
        Assert.True(3 * Square(delta) > 5 * Square(long.MaxValue));
        Assert.False(WideFiniteAxisIntersection.DoesCenteredFiniteCylindersPenetrate(
            first, firstRotation, size, size, second, secondRotation, size, size));
        Assert.False(WideFiniteAxisIntersection.DoesCenteredFiniteCylindersPenetrate(
            second, secondRotation, size, size, first, firstRotation, size, size));
        Assert.False(WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCone(
            first, firstRotation, size, size, second, secondRotation, size, size));
        Assert.False(WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCone(
            second, secondRotation, size, size, first, firstRotation, size, size));
    }

    private static bool ParallelCylinderOracle(Vector3d first, Vector3d second,
        long firstHeight, long secondHeight, long firstRadius, long secondRadius)
    {
        BigInteger x = (BigInteger)second.X.m_rawValue - first.X.m_rawValue;
        BigInteger y = (BigInteger)second.Y.m_rawValue - first.Y.m_rawValue;
        BigInteger z = (BigInteger)second.Z.m_rawValue - first.Z.m_rawValue;
        BigInteger axial = 4 * x - y + 8 * z;
        BigInteger height = (BigInteger)firstHeight + secondHeight;
        BigInteger radius = (BigInteger)firstRadius + secondRadius;
        // CommonFrame's exact +Y axis is (4,-1,8)/9. Clear denominators
        // directly; no production frame or distance helper participates.
        return 2 * BigInteger.Abs(axial) < 9 * height
            && 81 * (Square(x) + Square(y) + Square(z)) - Square(axial) < 81 * Square(radius);
    }

    private static bool ParallelCylinders(Vector3d first, Vector3d second,
        long firstHeight, long secondHeight, long firstRadius, long secondRadius) =>
        WideFiniteAxisIntersection.DoesCenteredFiniteCylindersPenetrate(
            first, CommonFrame, Fixed64.FromRaw(firstHeight), Fixed64.FromRaw(firstRadius),
            second, CommonFrame, Fixed64.FromRaw(secondHeight), Fixed64.FromRaw(secondRadius));

    private static Vector3d RawVector(long x, long y, long z) =>
        new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));

    private static bool Cylinders(Vector3d center, Fixed64 firstRadius, Fixed64 secondRadius, bool swap)
    {
        Vector3d transformed = TransformScaledCenter(center);
        return swap
            ? WideFiniteAxisIntersection.DoesCenteredFiniteCylindersPenetrate(
                transformed, PerpendicularFrame, (Fixed64)18, (Fixed64)9 * secondRadius,
                Translation, CommonFrame, (Fixed64)18, (Fixed64)9 * firstRadius)
            : WideFiniteAxisIntersection.DoesCenteredFiniteCylindersPenetrate(
                Translation, CommonFrame, (Fixed64)18, (Fixed64)9 * firstRadius,
                transformed, PerpendicularFrame, (Fixed64)18, (Fixed64)9 * secondRadius);
    }

    private static bool CylinderCone(Vector3d center, Fixed64 radius) =>
        WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCone(
            TransformScaledCenter(center), PerpendicularFrame, (Fixed64)18, (Fixed64)9 * radius,
            Translation, CommonFrame, (Fixed64)36, (Fixed64)27);

    private static Vector3d TransformScaledCenter(Vector3d point) => Translation + new Vector3d(
        -7 * point.X + 4 * point.Y + 4 * point.Z,
        4 * point.X - point.Y + 8 * point.Z,
        4 * point.X + 8 * point.Y - point.Z);

    private static bool CircleEntersTriangle(long x, long y, long radius,
        long ax, long ay, long bx, long by, long cx, long cy)
    {
        if (Cross(ax, ay, bx, by, x, y) <= 0 && Cross(bx, by, cx, cy, x, y) <= 0
            && Cross(cx, cy, ax, ay, x, y) <= 0)
            return true;
        return CircleEntersSegment(x, y, radius, ax, ay, bx, by)
            || CircleEntersSegment(x, y, radius, bx, by, cx, cy)
            || CircleEntersSegment(x, y, radius, cx, cy, ax, ay);
    }

    private static bool CircleEntersSegment(long x, long y, long radius, long ax, long ay, long bx, long by)
    {
        BigInteger dx = (BigInteger)bx - ax, dy = (BigInteger)by - ay;
        BigInteger wx = (BigInteger)x - ax, wy = (BigInteger)y - ay;
        BigInteger lengthSquared = Square(dx) + Square(dy);
        BigInteger parameter = wx * dx + wy * dy;
        if (parameter <= 0)
            return Square(wx) + Square(wy) < Square(radius);
        if (parameter >= lengthSquared)
            return Square((BigInteger)x - bx) + Square((BigInteger)y - by) < Square(radius);
        return Square(wx * dy - wy * dx) < Square(radius) * lengthSquared;
    }

    private static BigInteger Cross(long ax, long ay, long bx, long by, long x, long y) =>
        ((BigInteger)bx - ax) * ((BigInteger)y - ay) - ((BigInteger)by - ay) * ((BigInteger)x - ax);

    private static BigInteger Square(BigInteger value) => value * value;
}
