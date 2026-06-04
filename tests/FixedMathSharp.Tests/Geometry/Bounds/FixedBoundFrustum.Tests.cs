using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedBoundFrustumTests
{
    private static readonly FixedPlane CustomNear = new(Vector3d.Backward, Fixed64.One);
    private static readonly FixedPlane CustomFar = new(Vector3d.Forward, new Fixed64(-5));
    private static readonly FixedPlane CustomLeft = new(Vector3d.Left, new Fixed64(-2));
    private static readonly FixedPlane CustomRight = new(Vector3d.Right, new Fixed64(-2));
    private static readonly FixedPlane CustomTop = new(Vector3d.Up, new Fixed64(-3));
    private static readonly FixedPlane CustomBottom = new(Vector3d.Down, new Fixed64(-3));

    [Fact]
    public void Constructor_IdentityMatrix_CreatesExpectedClipSpaceBounds()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);

        Assert.Equal(new Vector3d(-1, -1, 0), frustum.Min);
        Assert.Equal(new Vector3d(1, 1, 1), frustum.Max);
    }

    [Fact]
    public void Constructor_Matrix_DoesNotAllocate()
    {
        _ = new FixedBoundFrustum(Fixed4x4.Identity);

        long before = GC.GetAllocatedBytesForCurrentThread();
        _ = new FixedBoundFrustum(Fixed4x4.Identity);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0, allocated);
    }

    [Fact]
    public void GetCorners_ReturnsCopyInStableOrder()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);

        Vector3d[] corners = frustum.GetCorners();

        Assert.Equal(new Vector3d(-1, 1, 0), corners[0]);
        Assert.Equal(new Vector3d(1, 1, 0), corners[1]);
        Assert.Equal(new Vector3d(1, -1, 0), corners[2]);
        Assert.Equal(new Vector3d(-1, -1, 0), corners[3]);
        Assert.Equal(new Vector3d(-1, 1, 1), corners[4]);
        Assert.Equal(new Vector3d(1, 1, 1), corners[5]);
        Assert.Equal(new Vector3d(1, -1, 1), corners[6]);
        Assert.Equal(new Vector3d(-1, -1, 1), corners[7]);

        corners[0] = Vector3d.Zero;

        Assert.Equal(new Vector3d(-1, 1, 0), frustum.GetCorners()[0]);
    }

    [Fact]
    public void Contains_Point_UsesAllSixPlanes()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);

        Assert.Equal(FixedEnclosureType.Contains, frustum.Contains(new Vector3d(0, 0, 0)));
        Assert.Equal(FixedEnclosureType.Contains, frustum.Contains(new Vector3d(1, 1, 1)));
        Assert.Equal(FixedEnclosureType.Disjoint, frustum.Contains(new Vector3d(0, 0, -1)));
        Assert.Equal(FixedEnclosureType.Disjoint, frustum.Contains(new Vector3d(2, 0, 0)));
        Assert.Equal(FixedEnclosureType.Disjoint, frustum.Contains(new Vector3d(0, 0, 2)));
    }

    [Fact]
    public void ClampPoint_ReturnsPointUnchangedWhenAlreadyInside()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var point = new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half);

        Assert.Equal(point, frustum.ClampPoint(point));
    }

    [Fact]
    public void ClampPoint_ClampsToFaceEdgeOrCorner()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);

        Assert.Equal(Vector3d.FromFloatPoint(1, 0, 0.5), frustum.ClampPoint(Vector3d.FromFloatPoint(2, 0, 0.5)));
        Assert.Equal(Vector3d.FromFloatPoint(1, 1, 0.5), frustum.ClampPoint(Vector3d.FromFloatPoint(2, 2, 0.5)));
        Assert.Equal(new Vector3d(1, 1, 1), frustum.ClampPoint(new Vector3d(2, 2, 2)));
    }

    [Fact]
    public void ClampPoint_UsesFrustumPlanesInsteadOfAabb()
    {
        var matrix = Fixed4x4.Identity;
        matrix.M31 = Fixed64.One;
        var frustum = new FixedBoundFrustum(matrix);
        var point = new Vector3d(1, 0, 1);

        Vector3d clamped = frustum.ClampPoint(point);

        Assert.NotEqual(point, clamped);
        Assert.Equal(FixedEnclosureType.Contains, frustum.Contains(clamped));
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.Half, clamped.X);
        Assert.Equal(Fixed64.Zero, clamped.Y);
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.Half, clamped.Z);
    }

    [Fact]
    public void Intersects_BoundingBox_ReturnsFalseOnlyWhenFullyOutsidePlane()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var inside = new FixedBoundBox(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        var outside = new FixedBoundBox(new Vector3d(new Fixed64(3), Fixed64.Zero, Fixed64.Half), new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        var crossing = new FixedBoundBox(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Half), new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));

        Assert.Equal(FixedEnclosureType.Contains, frustum.Contains(inside));
        Assert.True(frustum.Intersects(inside));
        Assert.False(frustum.Intersects(outside));
        Assert.True(frustum.Intersects(crossing));
        Assert.Equal(FixedEnclosureType.Intersects, frustum.Contains(crossing));
    }

    [Fact]
    public void Contains_BoundingArea_ReturnsContainsOrIntersects()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var inside = new FixedBoundArea(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFloatPoint(0.25)),
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.FromFloatPoint(0.75)));
        var crossing = new FixedBoundArea(
            new Vector3d(Fixed64.FromFloatPoint(0.75), -Fixed64.Half, Fixed64.FromFloatPoint(0.25)),
            new Vector3d(Fixed64.FromFloatPoint(1.25), Fixed64.Half, Fixed64.FromFloatPoint(0.75)));

        Assert.Equal(FixedEnclosureType.Contains, frustum.Contains(inside));
        Assert.Equal(FixedEnclosureType.Intersects, frustum.Contains(crossing));
    }

    [Fact]
    public void Intersects_BoundingSphere_ReturnsFalseOnlyWhenFullyOutsidePlane()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var inside = new FixedBoundSphere(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), Fixed64.FromFloatPoint(0.25));
        var outside = new FixedBoundSphere(new Vector3d(new Fixed64(3), Fixed64.Zero, Fixed64.Half), Fixed64.Half);
        var crossing = new FixedBoundSphere(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Half), Fixed64.Half);

        Assert.Equal(FixedEnclosureType.Contains, frustum.Contains(inside));
        Assert.True(frustum.Intersects(inside));
        Assert.False(frustum.Intersects(outside));
        Assert.True(frustum.Intersects(crossing));
        Assert.Equal(FixedEnclosureType.Intersects, frustum.Contains(crossing));
    }

    [Fact]
    public void Intersects_FixedPlane_ClassifiesAllCorners()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var frontPlane = new FixedPlane(Vector3d.Right, new Fixed64(2));
        var backPlane = new FixedPlane(Vector3d.Right, new Fixed64(-2));
        var crossingPlane = new FixedPlane(Vector3d.Right, Fixed64.Zero);
        var cornerPlane = new FixedPlane(Vector3d.Right, Fixed64.One);

        Assert.Equal(FixedPlaneIntersectionType.Front, frustum.Intersects(frontPlane));
        Assert.Equal(FixedPlaneIntersectionType.Back, frustum.Intersects(backPlane));
        Assert.Equal(FixedPlaneIntersectionType.Intersecting, frustum.Intersects(crossingPlane));
        Assert.Equal(FixedPlaneIntersectionType.Intersecting, frustum.Intersects(cornerPlane));
    }

    [Fact]
    public void Contains_BoundingFrustum_ReturnsContainsForSameOrNestedFrustum()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var nestedMatrix = Fixed4x4.Identity;
        nestedMatrix.M11 = new Fixed64(2);
        nestedMatrix.M22 = new Fixed64(2);
        var nested = new FixedBoundFrustum(nestedMatrix);

        Assert.Equal(FixedEnclosureType.Contains, frustum.Contains(frustum));
        Assert.Equal(FixedEnclosureType.Contains, frustum.Contains(nested));
        Assert.True(frustum.Intersects(nested));
    }

    [Fact]
    public void Contains_BoundingFrustum_ReturnsIntersectsForOverlappingFrustum()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var overlappingMatrix = Fixed4x4.Identity;
        overlappingMatrix.M41 = Fixed64.One;
        var overlapping = new FixedBoundFrustum(overlappingMatrix);

        Assert.Equal(FixedEnclosureType.Intersects, frustum.Contains(overlapping));
        Assert.True(frustum.Intersects(overlapping));
    }

    [Fact]
    public void Contains_BoundingFrustum_ReturnsDisjointForSeparatedFrustum()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var separatedMatrix = Fixed4x4.Identity;
        separatedMatrix.M41 = new Fixed64(3);
        var separated = new FixedBoundFrustum(separatedMatrix);

        Assert.Equal(FixedEnclosureType.Disjoint, frustum.Contains(separated));
        Assert.False(frustum.Intersects(separated));
    }

    [Fact]
    public void Contains_BoundingFrustum_ReturnsDisjointWhenOtherPlaneSeparates()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var other = CreateTransformedFrustum(
            new Vector3d(-4, -4, -2),
            new Vector3d(0, 45, 45),
            Vector3d.One);

        Assert.Equal(FixedEnclosureType.Disjoint, frustum.Contains(other));
        Assert.False(frustum.Intersects(other));
    }

    [Fact]
    public void Contains_BoundingFrustum_ReturnsDisjointForRotatedEdgeAxisSeparation()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var other = CreateTransformedFrustum(
            Vector3d.FromFloatPoint(1.336, 1.880, -0.396),
            Vector3d.FromFloatPoint(139.413, 115.507, 104.474),
            Vector3d.FromFloatPoint(1.660, 1.545, 2.338));

        Assert.Equal(FixedEnclosureType.Disjoint, frustum.Contains(other));
        Assert.False(frustum.Intersects(other));
    }

    [Fact]
    public void Matrix_Setter_RebuildsPlanesCornersAndBounds()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var point = new Vector3d(Fixed64.FromFloatPoint(0.75), Fixed64.Zero, Fixed64.Half);

        Assert.Equal(FixedEnclosureType.Contains, frustum.Contains(point));

        var matrix = Fixed4x4.Identity;
        matrix.M11 = new Fixed64(2);
        frustum.Matrix = matrix;

        Assert.Equal(FixedEnclosureType.Disjoint, frustum.Contains(point));
        Assert.Equal(new Vector3d(-Fixed64.Half, new Fixed64(-1), Fixed64.Zero), frustum.Min);
        Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One), frustum.Max);
    }

    [Fact]
    public void Constructor_Planes_CreatesExpectedFrustum()
    {
        var frustum = new FixedBoundFrustum(CustomNear, CustomFar, CustomLeft, CustomRight, CustomTop, CustomBottom);

        Assert.False(frustum.HasMatrix);
        Assert.Throws<InvalidOperationException>(() => frustum.Matrix);
        Assert.Equal(new Vector3d(-2, -3, 1), frustum.Min);
        Assert.Equal(new Vector3d(2, 3, 5), frustum.Max);
        Assert.Equal(FixedEnclosureType.Contains, frustum.Contains(new Vector3d(0, 0, 3)));
        Assert.Equal(FixedEnclosureType.Disjoint, frustum.Contains(new Vector3d(0, 0, 6)));

        Vector3d[] corners = frustum.GetCorners();

        Assert.Equal(new Vector3d(-2, 3, 1), corners[0]);
        Assert.Equal(new Vector3d(2, 3, 1), corners[1]);
        Assert.Equal(new Vector3d(2, -3, 1), corners[2]);
        Assert.Equal(new Vector3d(-2, -3, 1), corners[3]);
        Assert.Equal(new Vector3d(-2, 3, 5), corners[4]);
        Assert.Equal(new Vector3d(2, 3, 5), corners[5]);
        Assert.Equal(new Vector3d(2, -3, 5), corners[6]);
        Assert.Equal(new Vector3d(-2, -3, 5), corners[7]);
    }

    [Fact]
    public void Constructor_Planes_ThrowsWhenPlanesDoNotHaveUniqueCorners()
    {
        Assert.Throws<InvalidOperationException>(() => new FixedBoundFrustum(
            CustomNear,
            CustomFar,
            CustomNear,
            CustomRight,
            CustomTop,
            CustomBottom));
    }

    [Fact]
    public void Intersects_Ray_ReturnsNullWhenFrustumIsBehindRay()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var ray = new FixedRay(new Vector3d(0, 0, 2), Vector3d.Forward);

        Assert.Null(frustum.Intersects(ray));
    }

    [Fact]
    public void Constructor_PlaneArray_ValidatesLengthAndCopiesPlanes()
    {
        FixedPlane[] planes =
        {
            CustomNear,
            CustomFar,
            CustomLeft,
            CustomRight,
            CustomTop,
            CustomBottom
        };

        var frustum = new FixedBoundFrustum(planes);
        planes[0] = new FixedPlane(Vector3d.Forward, Fixed64.Zero);

        Assert.Equal(CustomNear, frustum.Near);
        Assert.Throws<ArgumentNullException>(() => new FixedBoundFrustum(null!));
        Assert.Throws<ArgumentException>(() => new FixedBoundFrustum(new FixedPlane[FixedBoundFrustum.PlaneCount - 1]));
    }

    [Fact]
    public void GetPlanes_ReturnsCopyInStableOrder()
    {
        var frustum = new FixedBoundFrustum(CustomNear, CustomFar, CustomLeft, CustomRight, CustomTop, CustomBottom);

        FixedPlane[] planes = frustum.GetPlanes();

        Assert.Equal(CustomNear, planes[0]);
        Assert.Equal(CustomFar, planes[1]);
        Assert.Equal(CustomLeft, planes[2]);
        Assert.Equal(CustomRight, planes[3]);
        Assert.Equal(CustomTop, planes[4]);
        Assert.Equal(CustomBottom, planes[5]);

        planes[0] = new FixedPlane(Vector3d.Forward, Fixed64.Zero);

        Assert.Equal(CustomNear, frustum.GetPlanes()[0]);

        var copied = new FixedPlane[FixedBoundFrustum.PlaneCount];
        frustum.GetPlanes(copied);

        Assert.Equal(CustomNear, copied[0]);
        Assert.Throws<ArgumentNullException>(() => frustum.GetPlanes(null!));
        Assert.Throws<ArgumentOutOfRangeException>(() => frustum.GetPlanes(new FixedPlane[FixedBoundFrustum.PlaneCount - 1]));
    }

    [Fact]
    public void GetPlanes_ArrayOverloadWithValidDestination_DoesNotAllocate()
    {
        var frustum = new FixedBoundFrustum(CustomNear, CustomFar, CustomLeft, CustomRight, CustomTop, CustomBottom);
        var copied = new FixedPlane[FixedBoundFrustum.PlaneCount];

        frustum.GetPlanes(copied);

        long before = GC.GetAllocatedBytesForCurrentThread();
        frustum.GetPlanes(copied);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0, allocated);
        Assert.Equal(CustomNear, copied[0]);
    }

    [Fact]
    public void GetCorners_ArrayOverloadCopiesAndValidatesDestination()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var copied = new Vector3d[FixedBoundFrustum.CornerCount];

        frustum.GetCorners(copied);

        Assert.Equal(frustum.GetCorners(), copied);
        Assert.Throws<ArgumentNullException>(() => frustum.GetCorners(null!));
        Assert.Throws<ArgumentOutOfRangeException>(() => frustum.GetCorners(new Vector3d[FixedBoundFrustum.CornerCount - 1]));
    }

    [Fact]
    public void GetCorners_ArrayOverloadWithValidDestination_DoesNotAllocate()
    {
        var frustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var copied = new Vector3d[FixedBoundFrustum.CornerCount];

        frustum.GetCorners(copied);

        long before = GC.GetAllocatedBytesForCurrentThread();
        frustum.GetCorners(copied);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0, allocated);
        Assert.Equal(new Vector3d(-1, 1, 0), copied[0]);
    }

    [Fact]
    public void Equality_ComparesFrustumShapeRatherThanMatrixSource()
    {
        var matrixFrustum = new FixedBoundFrustum(Fixed4x4.Identity);
        var planeFrustum = new FixedBoundFrustum(
            matrixFrustum.Near,
            matrixFrustum.Far,
            matrixFrustum.Left,
            matrixFrustum.Right,
            matrixFrustum.Top,
            matrixFrustum.Bottom);

        Assert.True(matrixFrustum.HasMatrix);
        Assert.Equal(Fixed4x4.Identity, matrixFrustum.Matrix);
        Assert.Equal(matrixFrustum, planeFrustum);
        Assert.True(matrixFrustum.Equals((object)planeFrustum));
        Assert.False(matrixFrustum.Equals((object)new object()));
        Assert.False(matrixFrustum.Equals(null));
        Assert.Equal(matrixFrustum.GetHashCode(), planeFrustum.GetHashCode());
    }

    private static FixedBoundFrustum CreateTransformedFrustum(Vector3d translation, Vector3d rotationDegrees, Vector3d scale)
    {
        Fixed4x4 translationMatrix = Fixed4x4.CreateTranslation(translation);
        Fixed4x4 rotationMatrix =
            Fixed4x4.CreateRotationX(rotationDegrees.X * Fixed64.Deg2Rad) *
            Fixed4x4.CreateRotationY(rotationDegrees.Y * Fixed64.Deg2Rad) *
            Fixed4x4.CreateRotationZ(rotationDegrees.Z * Fixed64.Deg2Rad);
        Fixed4x4 scaleMatrix = Fixed4x4.CreateScale(scale);

        return new FixedBoundFrustum(translationMatrix * rotationMatrix * scaleMatrix);
    }
}
