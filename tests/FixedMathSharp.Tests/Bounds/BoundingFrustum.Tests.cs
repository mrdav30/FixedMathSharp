using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class BoundingFrustumTests
{
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
}
