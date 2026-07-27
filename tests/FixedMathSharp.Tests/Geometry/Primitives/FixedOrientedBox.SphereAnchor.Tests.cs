//=======================================================================
// FixedOrientedBox.SphereAnchor.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxSphereAnchorTests
{
    [Fact]
    public void SphereContact_ShouldRetainObliqueSurfaceAsDeferredRadialFeature()
    {
        FixedQuaternion sphereRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)13,
                (Fixed64)(-27),
                (Fixed64)9);
        Vector3d sphereCenter = new(
            Fixed64.FromFraction(3, 2),
            Fixed64.Zero,
            Fixed64.FromFraction(3, 2));
        Fixed64 radius = Fixed64.One + Fixed64.MinIncrement;
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Assert.True(box.TryGetSphereContact(
            sphereCenter,
            sphereRotation,
            radius,
            out FixedContactAnchors contact));

        FixedPointAnchor surface = contact.SecondAnchor;
        Assert.Equal(Vector3d.Zero, surface.LocalPoint);
        Assert.NotEqual(Vector3d.Zero, surface.LocalDisplacement);
        var roundedOnly = new FixedPointAnchor(
            surface.Origin,
            surface.Rotation,
            surface.LocalPoint,
            surface.LocalDisplacement);
        Assert.NotEqual(0, surface.CompareLocalFeature(roundedOnly));

        var centerAnchor = new FixedPointAnchor(
            sphereCenter,
            sphereRotation,
            Vector3d.Zero);
        Assert.True(surface.TryGetProjectedOffsetFrom(
            centerAnchor,
            contact.Normal,
            out Fixed64 radiusProjection));
        Assert.True((radiusProjection + radius).Abs()
            <= Fixed64.Epsilon);
    }
}
