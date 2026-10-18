using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WideTriangleRelationsFilteredContactsTests
{
    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void SignedResponseAdmission_ShouldRetainTheAllowedExitInsteadOfRejectingTheOriginalWinner(int sign)
    {
        FixedTriangle moving = new(new Vector3d(Fixed64.One, Fixed64.One, -Fixed64.Quarter),
            new Vector3d(1, 1, 1), new Vector3d(3, 1, 1));
        FixedTriangle sheet = new(new Vector3d(0, 0, 0), new Vector3d(4, 0, 0), new Vector3d(0, 4, 0));
        Assert.True(WideTriangleRelations.TryGetPenetration(moving, Vector3d.Zero, FixedQuaternion.Identity,
            sheet, Vector3d.Zero, FixedQuaternion.Identity, out var result, Admit));
        Assert.True(result.HasValue);
        WideAxis3 axis = result.Negate ? -result.Axis : result.Axis;
        Assert.True(axis.X.IsZero && axis.Y.IsZero); Assert.Equal(sign, axis.Z.Sign);
        Fixed64 depth = result.GetRoundedDepth(out bool clamped);
        Assert.False(clamped); Assert.Equal(sign > 0 ? Fixed64.One : Fixed64.Quarter, depth);
        Vector3d outward = Vector3d.Forward * (Fixed64)(-sign);
        Assert.True(WideTriangleRelations.TryGetPenetration(moving, outward * depth, FixedQuaternion.Identity,
            sheet, Vector3d.Zero, FixedQuaternion.Identity, out _, Admit));
        Assert.False(WideTriangleRelations.TryGetPenetration(moving, outward * (depth + Fixed64.MinIncrement),
            FixedQuaternion.Identity, sheet, Vector3d.Zero, FixedQuaternion.Identity, out _, Admit));

        bool Admit(in WidePointSpanPenetration candidate)
        {
            WideAxis3 direction = candidate.Negate ? -candidate.Axis : candidate.Axis;
            return direction.X.IsZero && direction.Y.IsZero && direction.Z.Sign == sign;
        }
    }

    [Fact]
    public void ExcludingEveryResponse_ShouldPreserveGeometricOverlap()
    {
        FixedTriangle first = new(new Vector3d(0, 0, 0), new Vector3d(4, 0, 0), new Vector3d(0, 4, 0));
        FixedTriangle second = new(new Vector3d(1, 1, 0), new Vector3d(3, 1, 0), new Vector3d(1, 3, 0));
        Assert.True(WideTriangleRelations.TryGetPenetration(first, Vector3d.Zero, FixedQuaternion.Identity,
            second, Vector3d.Zero, FixedQuaternion.Identity, out var result, Reject));
        Assert.False(result.HasValue);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ExcludingEveryResponse_ShouldStillRejectOwnedInPlaneSeparators(bool secondEdge, bool reverse)
    {
        FixedTriangle first = new(new Vector3d(0, 0, 0), new Vector3d(4, 0, 0), new Vector3d(0, 4, 0));
        FixedTriangle second = secondEdge
            ? new(new Vector3d(3, -2, 0), new Vector3d(5, 0, 0), new Vector3d(5, -1, 0))
            : new(new Vector3d(3, 3, 0), new Vector3d(5, 3, 0), new Vector3d(3, 5, 0));
        if (reverse) (first, second) = (second, first);
        Assert.False(WideTriangleRelations.TryGetPenetration(first, Vector3d.Zero, FixedQuaternion.Identity,
            second, Vector3d.Zero, FixedQuaternion.Identity, out _, Reject));
    }

    private static bool Reject(in WidePointSpanPenetration _) => false;
}
