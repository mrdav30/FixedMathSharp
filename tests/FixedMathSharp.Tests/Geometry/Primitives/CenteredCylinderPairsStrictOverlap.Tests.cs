using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderPairsStrictOverlapTests
{
    private static readonly Fixed64 QuarterComponent = Fixed64.FromRaw(3_037_000_500L);
    private static readonly FixedQuaternion AlongX = new(Fixed64.Zero, Fixed64.Zero, -QuarterComponent, QuarterComponent);

    [Theory]
    [InlineData(false, -1L, true)]
    [InlineData(false, 0L, false)]
    [InlineData(false, 1L, false)]
    [InlineData(true, -1L, true)]
    [InlineData(true, 0L, false)]
    [InlineData(true, 1L, false)]
    public void Cylinders_ClassifyExactRimBoundaryInEitherOrder(bool swap, long offset, bool expected)
    {
        Vector3d second = new((Fixed64)7 / 4, (Fixed64)7 / 4, Fixed64.Two + Fixed64.FromRaw(offset));
        Fixed64 radius = (Fixed64)5 / 4;
        Assert.Equal(expected, swap
            ? Cylinders(second, AlongX, radius, Vector3d.Zero, FixedQuaternion.Identity, radius)
            : Cylinders(Vector3d.Zero, FixedQuaternion.Identity, radius, second, AlongX, radius));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void Cylinders_RejectMirroredSeparatedRims(int mirror) =>
        Assert.False(Cylinders(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One,
            new Vector3d((Fixed64)7 / 4, (Fixed64)7 / 4, mirror * (Fixed64)11 / 8), AlongX, Fixed64.One));

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void Cylinders_ClassifyInteriorSideBoundary(long offset, bool expected) =>
        Assert.Equal(expected, Cylinders(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Two + Fixed64.FromRaw(offset)), AlongX, Fixed64.One));

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void Cylinders_ClassifyParallelAxialBoundary(long offset, bool expected) =>
        Assert.Equal(expected, Cylinders(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One,
            new Vector3d(Fixed64.Zero, Fixed64.Two + Fixed64.FromRaw(offset), Fixed64.Zero),
            FixedQuaternion.Identity, Fixed64.One));

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void Cylinders_PreserveFullDomainTranslatedRimBoundary(long offset, bool expected)
    {
        Vector3d first = new(Fixed64.MaxValue - (Fixed64)8, Fixed64.MinValue + (Fixed64)8, Fixed64.MaxValue - (Fixed64)8);
        Vector3d second = first + new Vector3d((Fixed64)7 / 4, (Fixed64)7 / 4,
            Fixed64.Two + Fixed64.FromRaw(offset));
        Assert.Equal(expected, Cylinders(first, FixedQuaternion.Identity, (Fixed64)5 / 4,
            second, AlongX, (Fixed64)5 / 4));
    }

    [Fact]
    public void Cylinders_ContainmentUsesAuthoritativeArbitraryRigidFrame()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)13, (Fixed64)29, (Fixed64)41);
        Vector3d center = new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue);
        Assert.True(WideFiniteAxisIntersection.DoesCenteredFiniteCylindersPenetrate(
            center, rotation, Fixed64.One, Fixed64.One,
            center, rotation, (Fixed64)4, (Fixed64)3));
    }

    [Theory]
    [InlineData(-1, -1L, false)]
    [InlineData(-1, 0L, false)]
    [InlineData(-1, 1L, true)]
    [InlineData(1, -1L, false)]
    [InlineData(1, 0L, false)]
    [InlineData(1, 1L, true)]
    public void CylinderCone_ClassifiesMirroredInteriorSideBoundary(int mirror, long radiusOffset, bool expected)
    {
        // H=4,R=3 gives side unit normal (0,3/5,4/5). The cylinder
        // center is five units outside the generator midpoint (0,0,3/2).
        Assert.Equal(expected, Cone(new Vector3d(Fixed64.Zero, (Fixed64)3, mirror * (Fixed64)11 / 2),
            AlongX, (Fixed64)5 + Fixed64.FromRaw(radiusOffset)));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void CylinderCone_RejectsInfiniteAxisWinnerOutsideCylinderCaps(int direction) =>
        Assert.False(Cone(new Vector3d(direction * (Fixed64)2, (Fixed64)3, (Fixed64)11 / 2),
            AlongX, (Fixed64)5 + Fixed64.FromRaw(1)));

    [Fact]
    public void CylinderCone_RejectsSeparatedParallelAxes() =>
        Assert.False(Cone(new Vector3d(4, 0, 0), FixedQuaternion.Identity, Fixed64.One));

    [Fact]
    public void CylinderCone_RejectsGeneratorWinnerBelowBase() =>
        Assert.False(Cone(new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)31 / 4),
            AlongX, (Fixed64)5 + Fixed64.FromRaw(1)));

    [Fact]
    public void CylinderCone_RejectsGeneratorWinnerAboveApex() =>
        Assert.False(Cone(new Vector3d(Fixed64.Zero, (Fixed64)6, (Fixed64)13 / 4),
            AlongX, (Fixed64)5 + Fixed64.FromRaw(1)));

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, false)]
    [InlineData(1L, true)]
    public void CylinderCone_ClassifiesExactParallelGeneratorBoundary(long offset, bool expected)
    {
        // Quaternion ratio -3:1 gives the exact rational axis (3/5,-4/5,0),
        // parallel to the cone generator. The center is five units outside it.
        Fixed64 q = Fixed64.FromRaw(1_358_187_913L);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, -3 * q, q);
        Assert.True(rotation.IsNormalized());
        Assert.Equal(expected, Cone(new Vector3d((Fixed64)11 / 2, (Fixed64)3, Fixed64.Zero),
            rotation, (Fixed64)5 + Fixed64.FromRaw(offset)));
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, false)]
    [InlineData(1L, true)]
    public void CylinderCone_ClassifiesReversedParallelGeneratorBoundary(long offset, bool expected)
    {
        Fixed64 q = Fixed64.FromRaw(1_358_187_913L);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, q, 3 * q);
        Assert.True(rotation.IsNormalized());
        Assert.Equal(expected, Cone(new Vector3d((Fixed64)11 / 2, (Fixed64)3, Fixed64.Zero),
            rotation, (Fixed64)5 + Fixed64.FromRaw(offset)));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void CylinderCone_RejectsParallelGeneratorBeyondEitherCylinderCap(int direction)
    {
        Fixed64 q = Fixed64.FromRaw(1_358_187_913L);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, -3 * q, q);
        // Translate ten units along the exact (3/5,-4/5,0) cylinder axis.
        // The infinite-line distance is still below the radius, but neither
        // finite segment reaches the cone's generator interval.
        Vector3d center = new((Fixed64)11 / 2 + direction * 6, (Fixed64)3 - direction * 8, Fixed64.Zero);
        Assert.False(Cone(center, rotation, (Fixed64)5 + Fixed64.FromRaw(1)));
    }

    [Fact]
    public void CylinderCone_DetectsContainedApex()
    {
        Assert.True(WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCone(
            new Vector3d(0, 2, 0), FixedQuaternion.Identity, Fixed64.One, Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)3));
    }

    [Fact]
    public void CylinderCone_UsesAuthoritativeArbitraryRigidFrame()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)13, (Fixed64)29, (Fixed64)41);
        Vector3d center = new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue);
        Assert.True(WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCone(
            center, rotation, Fixed64.One, Fixed64.One,
            center, rotation, (Fixed64)4, (Fixed64)3));
    }

    [Fact]
    public void Cylinders_MatchIndependentPerpendicularDiskIntervalOracle()
    {
        for (int sample = 0; sample < 128; sample++)
        {
            Fixed64 firstRadius = Fixed64.One + (Fixed64)(sample % 3) / 4;
            Fixed64 secondRadius = Fixed64.One + (Fixed64)(sample / 3 % 3) / 4;
            Vector3d center = new((Fixed64)(sample * 13 % 65 - 32) / 8,
                (Fixed64)(sample * 19 % 65 - 32) / 8, (Fixed64)(sample * 29 % 65 - 32) / 8);
            BigInteger x = BigInteger.Max(BigInteger.Abs(center.X.m_rawValue) - Fixed64.One.m_rawValue, 0);
            BigInteger y = BigInteger.Max(BigInteger.Abs(center.Y.m_rawValue) - Fixed64.One.m_rawValue, 0);
            BigInteger a = Square(firstRadius.m_rawValue) - x * x;
            BigInteger b = Square(secondRadius.m_rawValue) - y * y;
            BigInteger difference = Square(center.Z.m_rawValue) - a - b;
            bool expected = a > 0 && b > 0 && (difference < 0 || difference * difference < 4 * a * b);
            bool actual = Cylinders(Vector3d.Zero, FixedQuaternion.Identity, firstRadius, center, AlongX, secondRadius);
            bool swapped = Cylinders(center, AlongX, secondRadius, Vector3d.Zero, FixedQuaternion.Identity, firstRadius);
            Assert.True(actual == expected && swapped == expected,
                $"Perpendicular interval sample {sample}: expected {expected}, actual {actual}, swapped {swapped}.");
        }
    }

    [Fact]
    public void CylinderCone_MatchesIndependentMeridianTriangleOracle()
    {
        long halfHeight = ((Fixed64)2).m_rawValue;
        long radius = ((Fixed64)3).m_rawValue;
        for (int sample = 0; sample < 128; sample++)
        {
            Fixed64 y = (Fixed64)(sample * 17 % 81 - 40) / 8;
            Fixed64 z = (Fixed64)(sample * 11 % 81 - 40) / 8;
            Fixed64 cylinderRadius = Fixed64.Half + (Fixed64)(sample % 5) / 4;
            // The cylinder axis is X and its center has x=0. Every feasible
            // point can be moved to x=0 without worsening either inequality,
            // so the problem is exactly a circle against this YZ triangle.
            bool expected = CircleEntersTriangle(y.m_rawValue, z.m_rawValue, cylinderRadius.m_rawValue,
                -halfHeight, -radius, -halfHeight, radius, halfHeight, 0);
            bool actual = Cone(new Vector3d(Fixed64.Zero, y, z), AlongX, cylinderRadius);
            Assert.True(actual == expected, $"Meridian sample {sample}: expected {expected}, actual {actual}.");
        }
    }

    [Fact]
    public void StrictPairs_AllocateNothingAfterWarmup()
    {
        Vector3d rimCenter = new((Fixed64)7 / 4, (Fixed64)7 / 4, Fixed64.Two - Fixed64.FromRaw(1));
        Vector3d coneSideCenter = new(Fixed64.Zero, (Fixed64)3, (Fixed64)11 / 2);
        Fixed64 radius = (Fixed64)5 + Fixed64.FromRaw(1);
        bool result = true;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            for (int index = 0; index < 32; index++)
            {
                result &= Cylinders(Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)5 / 4,
                    rimCenter, AlongX, (Fixed64)5 / 4);
                result &= Cone(coneSideCenter, AlongX, radius);
            }
        });
        Assert.True(result);
        Assert.Equal(0L, allocated);
    }

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
        BigInteger lengthSquared = dx * dx + dy * dy;
        BigInteger parameter = wx * dx + wy * dy;
        if (parameter <= 0)
            return wx * wx + wy * wy < Square(radius);
        if (parameter >= lengthSquared)
            return Square((BigInteger)x - bx) + Square((BigInteger)y - by) < Square(radius);
        BigInteger cross = wx * dy - wy * dx;
        return cross * cross < Square(radius) * lengthSquared;
    }

    private static BigInteger Cross(long ax, long ay, long bx, long by, long x, long y) =>
        ((BigInteger)bx - ax) * ((BigInteger)y - ay) - ((BigInteger)by - ay) * ((BigInteger)x - ax);

    private static BigInteger Square(BigInteger value) => value * value;

    private static bool Cylinders(Vector3d first, FixedQuaternion firstRotation, Fixed64 firstRadius,
        Vector3d second, FixedQuaternion secondRotation, Fixed64 secondRadius) =>
        WideFiniteAxisIntersection.DoesCenteredFiniteCylindersPenetrate(first, firstRotation, Fixed64.Two, firstRadius,
            second, secondRotation, Fixed64.Two, secondRadius);

    private static bool Cone(Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Fixed64 radius) =>
        WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCone(
            cylinderCenter, cylinderRotation, Fixed64.Two, radius,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)3);
}
