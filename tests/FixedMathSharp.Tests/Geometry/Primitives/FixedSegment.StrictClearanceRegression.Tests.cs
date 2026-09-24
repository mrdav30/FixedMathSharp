using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedSegmentStrictClearanceRegressionTests
{
    [Fact]
    public void CylinderCapsuleContact_RejectsSeparatedEndpointBeyondCylinderRim()
    {
        // The closest core endpoint is (10.75, 1.75, 0), and the closest
        // cylinder point is (10, 1, 0): squared distance is 9/8 > radius^2.
        // Cylinder/capsule axes and the closest core-pair direction alone
        // miss the separating endpoint-to-rim direction (0.75, 0.75, 0).
        Fixed64 threeQuarters = (Fixed64)3 / 4;
        Vector3d capsuleCenter = new(
            (Fixed64)20 + threeQuarters,
            Fixed64.One + threeQuarters,
            Fixed64.Zero);

        Assert.False(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            (Fixed64)10,
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Right,
            (Fixed64)20,
            Fixed64.One,
            out _));
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void CylinderCapsuleContact_ClassifiesEndpointRimTangencyExactly(
        long verticalRawOffset,
        bool expectedContact)
    {
        // At zero offset, the rim distance is exactly sqrt((3/4)^2 + 1)
        // = 5/4. Changing the vertical gap by u changes squared distance
        // by 2*u + u^2, whose sign is the sign of these +/-1-raw offsets.
        Fixed64 threeQuarters = (Fixed64)3 / 4;
        Vector3d capsuleCenter = new(
            (Fixed64)20 + threeQuarters,
            Fixed64.Two + Fixed64.FromRaw(verticalRawOffset),
            Fixed64.Zero);

        bool hasContact = FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            (Fixed64)10,
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Right,
            (Fixed64)20,
            (Fixed64)5 / 4,
            out FixedContactAnchors contact);

        Assert.Equal(expectedContact, hasContact);
        if (verticalRawOffset == 0)
            Assert.Equal(Fixed64.Zero, contact.Depth);
    }

    [Fact]
    public void CylinderCapsuleContact_AdmitsPenetratingEndpointBeyondCylinderRim()
    {
        // The same exact 9/8 squared rim distance is now below (5/4)^2.
        Fixed64 threeQuarters = (Fixed64)3 / 4;
        Vector3d capsuleCenter = new(
            (Fixed64)20 + threeQuarters,
            Fixed64.One + threeQuarters,
            Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            (Fixed64)10,
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Right,
            (Fixed64)20,
            (Fixed64)5 / 4,
            out FixedContactAnchors contact));
        Assert.True(contact.Depth > Fixed64.Zero);
    }
}
