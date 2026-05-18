using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class BoundingFrustumTests
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
        var frustum = new BoundingFrustum(Fixed4x4.Identity);

        Assert.Equal(new Vector3d(-1, -1, 0), frustum.Min);
        Assert.Equal(new Vector3d(1, 1, 1), frustum.Max);
    }

    [Fact]
    public void GetCorners_ReturnsCopyInStableOrder()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);

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
        var frustum = new BoundingFrustum(Fixed4x4.Identity);

        Assert.Equal(ContainmentType.Contains, frustum.Contains(new Vector3d(0, 0, 0)));
        Assert.Equal(ContainmentType.Contains, frustum.Contains(new Vector3d(1, 1, 1)));
        Assert.Equal(ContainmentType.Disjoint, frustum.Contains(new Vector3d(0, 0, -1)));
        Assert.Equal(ContainmentType.Disjoint, frustum.Contains(new Vector3d(2, 0, 0)));
        Assert.Equal(ContainmentType.Disjoint, frustum.Contains(new Vector3d(0, 0, 2)));
    }

    [Fact]
    public void ClampPoint_ReturnsPointUnchangedWhenAlreadyInside()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);
        var point = new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half);

        Assert.Equal(point, frustum.ClampPoint(point));
    }

    [Fact]
    public void ClampPoint_ClampsToFaceEdgeOrCorner()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);

        Assert.Equal(new Vector3d(1, 0, 0.5), frustum.ClampPoint(new Vector3d(2, 0, 0.5)));
        Assert.Equal(new Vector3d(1, 1, 0.5), frustum.ClampPoint(new Vector3d(2, 2, 0.5)));
        Assert.Equal(new Vector3d(1, 1, 1), frustum.ClampPoint(new Vector3d(2, 2, 2)));
    }

    [Fact]
    public void ClampPoint_UsesFrustumPlanesInsteadOfAabb()
    {
        var matrix = Fixed4x4.Identity;
        matrix.m20 = Fixed64.One;
        var frustum = new BoundingFrustum(matrix);
        var point = new Vector3d(1, 0, 1);

        Vector3d clamped = frustum.ClampPoint(point);

        Assert.NotEqual(point, clamped);
        Assert.Equal(ContainmentType.Contains, frustum.Contains(clamped));
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.Half, clamped.x);
        Assert.Equal(Fixed64.Zero, clamped.y);
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.Half, clamped.z);
    }

    [Fact]
    public void Intersects_BoundingBox_ReturnsFalseOnlyWhenFullyOutsidePlane()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);
        var inside = new BoundingBox(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        var outside = new BoundingBox(new Vector3d(new Fixed64(3), Fixed64.Zero, Fixed64.Half), new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        var crossing = new BoundingBox(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Half), new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));

        Assert.Equal(ContainmentType.Contains, frustum.Contains(inside));
        Assert.True(frustum.Intersects(inside));
        Assert.False(frustum.Intersects(outside));
        Assert.True(frustum.Intersects(crossing));
        Assert.Equal(ContainmentType.Intersects, frustum.Contains(crossing));
    }

    [Fact]
    public void Intersects_BoundingSphere_ReturnsFalseOnlyWhenFullyOutsidePlane()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);
        var inside = new BoundingSphere(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), new Fixed64(0.25));
        var outside = new BoundingSphere(new Vector3d(new Fixed64(3), Fixed64.Zero, Fixed64.Half), Fixed64.Half);
        var crossing = new BoundingSphere(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Half), Fixed64.Half);

        Assert.Equal(ContainmentType.Contains, frustum.Contains(inside));
        Assert.True(frustum.Intersects(inside));
        Assert.False(frustum.Intersects(outside));
        Assert.True(frustum.Intersects(crossing));
        Assert.Equal(ContainmentType.Intersects, frustum.Contains(crossing));
    }

    [Fact]
    public void Intersects_FixedPlane_ClassifiesAllCorners()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);
        var frontPlane = new FixedPlane(Vector3d.Right, new Fixed64(2));
        var backPlane = new FixedPlane(Vector3d.Right, new Fixed64(-2));
        var crossingPlane = new FixedPlane(Vector3d.Right, Fixed64.Zero);

        Assert.Equal(FixedPlaneIntersectionType.Front, frustum.Intersects(frontPlane));
        Assert.Equal(FixedPlaneIntersectionType.Back, frustum.Intersects(backPlane));
        Assert.Equal(FixedPlaneIntersectionType.Intersecting, frustum.Intersects(crossingPlane));
    }

    [Fact]
    public void Contains_BoundingFrustum_ReturnsContainsForSameOrNestedFrustum()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);
        var nestedMatrix = Fixed4x4.Identity;
        nestedMatrix.m00 = new Fixed64(2);
        nestedMatrix.m11 = new Fixed64(2);
        var nested = new BoundingFrustum(nestedMatrix);

        Assert.Equal(ContainmentType.Contains, frustum.Contains(frustum));
        Assert.Equal(ContainmentType.Contains, frustum.Contains(nested));
        Assert.True(frustum.Intersects(nested));
    }

    [Fact]
    public void Contains_BoundingFrustum_NullFrustum_Throws()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);

        Assert.Throws<ArgumentNullException>(() => frustum.Contains((BoundingFrustum)null!));
    }

    [Fact]
    public void Contains_BoundingFrustum_ReturnsIntersectsForOverlappingFrustum()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);
        var overlappingMatrix = Fixed4x4.Identity;
        overlappingMatrix.m30 = Fixed64.One;
        var overlapping = new BoundingFrustum(overlappingMatrix);

        Assert.Equal(ContainmentType.Intersects, frustum.Contains(overlapping));
        Assert.True(frustum.Intersects(overlapping));
    }

    [Fact]
    public void Contains_BoundingFrustum_ReturnsDisjointForSeparatedFrustum()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);
        var separatedMatrix = Fixed4x4.Identity;
        separatedMatrix.m30 = new Fixed64(3);
        var separated = new BoundingFrustum(separatedMatrix);

        Assert.Equal(ContainmentType.Disjoint, frustum.Contains(separated));
        Assert.False(frustum.Intersects(separated));
    }

    [Fact]
    public void Matrix_Setter_RebuildsPlanesCornersAndBounds()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);
        var point = new Vector3d(new Fixed64(0.75), Fixed64.Zero, Fixed64.Half);

        Assert.Equal(ContainmentType.Contains, frustum.Contains(point));

        var matrix = Fixed4x4.Identity;
        matrix.m00 = new Fixed64(2);
        frustum.Matrix = matrix;

        Assert.Equal(ContainmentType.Disjoint, frustum.Contains(point));
        Assert.Equal(new Vector3d(-Fixed64.Half, new Fixed64(-1), Fixed64.Zero), frustum.Min);
        Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One), frustum.Max);
    }

    [Fact]
    public void Constructor_Planes_CreatesExpectedFrustum()
    {
        var frustum = new BoundingFrustum(CustomNear, CustomFar, CustomLeft, CustomRight, CustomTop, CustomBottom);

        Assert.False(frustum.HasMatrix);
        Assert.Throws<InvalidOperationException>(() => frustum.Matrix);
        Assert.Equal(new Vector3d(-2, -3, 1), frustum.Min);
        Assert.Equal(new Vector3d(2, 3, 5), frustum.Max);
        Assert.Equal(ContainmentType.Contains, frustum.Contains(new Vector3d(0, 0, 3)));
        Assert.Equal(ContainmentType.Disjoint, frustum.Contains(new Vector3d(0, 0, 6)));

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

        var frustum = new BoundingFrustum(planes);
        planes[0] = new FixedPlane(Vector3d.Forward, Fixed64.Zero);

        Assert.Equal(CustomNear, frustum.Near);
        Assert.Throws<ArgumentNullException>(() => new BoundingFrustum(null!));
        Assert.Throws<ArgumentException>(() => new BoundingFrustum(new FixedPlane[BoundingFrustum.PlaneCount - 1]));
    }

    [Fact]
    public void GetPlanes_ReturnsCopyInStableOrder()
    {
        var frustum = new BoundingFrustum(CustomNear, CustomFar, CustomLeft, CustomRight, CustomTop, CustomBottom);

        FixedPlane[] planes = frustum.GetPlanes();

        Assert.Equal(CustomNear, planes[0]);
        Assert.Equal(CustomFar, planes[1]);
        Assert.Equal(CustomLeft, planes[2]);
        Assert.Equal(CustomRight, planes[3]);
        Assert.Equal(CustomTop, planes[4]);
        Assert.Equal(CustomBottom, planes[5]);

        planes[0] = new FixedPlane(Vector3d.Forward, Fixed64.Zero);

        Assert.Equal(CustomNear, frustum.GetPlanes()[0]);

        var copied = new FixedPlane[BoundingFrustum.PlaneCount];
        frustum.GetPlanes(copied);

        Assert.Equal(CustomNear, copied[0]);
        Assert.Throws<ArgumentNullException>(() => frustum.GetPlanes(null!));
        Assert.Throws<ArgumentOutOfRangeException>(() => frustum.GetPlanes(new FixedPlane[BoundingFrustum.PlaneCount - 1]));
    }

    [Fact]
    public void GetCorners_ArrayOverloadCopiesAndValidatesDestination()
    {
        var frustum = new BoundingFrustum(Fixed4x4.Identity);
        var copied = new Vector3d[BoundingFrustum.CornerCount];

        frustum.GetCorners(copied);

        Assert.Equal(frustum.GetCorners(), copied);
        Assert.Throws<ArgumentNullException>(() => frustum.GetCorners(null!));
        Assert.Throws<ArgumentOutOfRangeException>(() => frustum.GetCorners(new Vector3d[BoundingFrustum.CornerCount - 1]));
    }

    [Fact]
    public void Equality_ComparesFrustumShapeRatherThanMatrixSource()
    {
        var matrixFrustum = new BoundingFrustum(Fixed4x4.Identity);
        var planeFrustum = new BoundingFrustum(
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
}
