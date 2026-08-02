//=======================================================================
// WideRigidProjection.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class WideRigidProjectionTests
{
    [Fact]
    public void PreparedLocalAxis_MatchesRotatedPointProjection()
    {
        var basis = new WideRationalBasis3d(
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)11,
                (Fixed64)(-23),
                (Fixed64)37));
        var axis = new WideAxis3(
            Signed320.ExtendValue(Signed192.Raw((Fixed64)7)),
            Signed320.ExtendValue(Signed192.Raw((Fixed64)(-11))),
            Signed320.ExtendValue(Signed192.Raw((Fixed64)13)));
        var point = new Vector3d(3, -2, 5);

        WideRigidProjection.GetLocalAxisProjections(
            axis,
            basis,
            out Signed576 x,
            out Signed576 y,
            out Signed576 z);
        Signed576 projection = WideRigidProjection.GetLocalOffsetProjection(
            point,
            x,
            y,
            z);
        WideAxis3 transformedPoint = WideRigidProjection.TransformLocalAxis(
            basis,
            Signed192.Raw(point.X),
            Signed192.Raw(point.Y),
            Signed192.Raw(point.Z));

        Assert.Equal(WideAxis3.Dot(axis, transformedPoint), projection);
    }
}
