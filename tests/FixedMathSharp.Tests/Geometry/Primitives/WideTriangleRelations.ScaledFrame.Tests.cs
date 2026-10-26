using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class WideTriangleRelationsScaledFrameTests
{
    private static readonly FixedTriangle Face = new(new(0, 0, 0), new(4, 0, 0), new(0, 0, 4));

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FractionalRotatedFace_ShouldAdmitExactFiniteProjectionAndRoundOnlyItsFinalWitness(bool compound)
    {
        FixedQuaternion geometry = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)45);
        FixedQuaternion placement = compound ? FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)(-11), (Fixed64)7, (Fixed64)31) : geometry;
        Vector3d reference = new(2, 0, 2), origin = new(-3, 5, 7);
        ScaledRigidFrame3d frame = new(origin, placement, reference, new((Fixed64)2 / 3, Fixed64.One, (Fixed64)3 / 5),
            geometry, reference, Vector3d.One);
        RationalPoint3d point = frame.GetPoint(new(1, 1, 1));
        Assert.True(WideTriangleRelations.ContainsProjection(Face, frame, point));
        Assert.True(WideTriangleRelations.TryGetProjectedPointAnchor(Face, frame, point, origin, out var anchor));
        RationalPoint3d expected = frame.GetPoint(new(1, 0, 1));
        Assert.True(expected.TryGetLocalPointIn(origin, FixedQuaternion.Identity, out var local));
        Assert.Equal(origin, anchor.Origin);
        Assert.Equal(local, anchor.LocalPoint);
        Assert.True(WideTriangleRelations.ContainsProjection(Face, frame, frame.GetPoint(new(2, 1, 2))));
        Assert.False(WideTriangleRelations.ContainsProjection(Face, frame, frame.GetPoint(new(4, 1, 4))));
        Assert.False(WideTriangleRelations.TryGetProjectedPointAnchor(Face, frame,
            frame.GetPoint(new(4, 1, 4)), origin, out _));
    }

    [Fact]
    public void ProjectedSegment_ShouldClipItsExactUnitParameterBeforeMaterialization()
    {
        ScaledRigidFrame3d frame = new(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new((Fixed64)2 / 3, Fixed64.One, (Fixed64)3 / 5), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        RationalPoint3d first = frame.GetPoint(new(-2, 1, 1)), second = frame.GetPoint(new(6, 1, 1));
        Assert.True(WideTriangleRelations.TryGetProjectedSegmentInterval(Face, frame, first, second,
            out var lowerN, out var lowerD, out var upperN, out var upperD));
        Assert.Equal(0, WideArithmetic.CompareNonNegativeProducts(lowerN, Signed832.ExtendValue(Signed192.Signed(4)),
            Signed832.ExtendValue(Signed192.Signed(1)), lowerD));
        Assert.Equal(0, WideArithmetic.CompareNonNegativeProducts(upperN, Signed832.ExtendValue(Signed192.Signed(8)),
            Signed832.ExtendValue(Signed192.Signed(5)), upperD));
        Assert.False(WideTriangleRelations.TryGetProjectedSegmentInterval(Face, frame,
            frame.GetPoint(new(-2, 1, -1)), frame.GetPoint(new(6, 1, -1)), out _, out _, out _, out _));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ProjectedRectangle_ShouldRetainExactTangentCutsAndBothHeightLimits(bool affine, bool reverse)
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)31);
        Vector3d scale = affine ? new(Fixed64.Half, Fixed64.Two, Fixed64.One) : Vector3d.One;
        var frame = new ScaledRigidFrame3d(new(20, -7, 11), rotation, Vector3d.Zero,
            scale, rotation, Vector3d.Zero, Vector3d.One);
        var rectangle = new FixedTriangle(new(-2, -1, 0), new(2, -1, 0), new(-2, 1, 0));
        RationalPoint3d a = frame.GetPoint(new(-3, 0, 3)), b = frame.GetPoint(new(3, 0, 3));
        var cut = new RationalSegmentInterval(L(1), L(2), L(1), L(1));
        const int words = 32;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var lowerN = ContactQuadratic.At(values, signs, 0, words);
        var lowerD = ContactQuadratic.At(values, signs, 1, words);
        var upperN = ContactQuadratic.At(values, signs, 2, words);
        var upperD = ContactQuadratic.At(values, signs, 3, words);
        Assert.True(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            reverse ? b : a, reverse ? a : b, cut, lowerN, lowerD, upperN, upperD));
        AssertRectangleBound(lowerN, lowerD, 1, reverse ? 6 : 2);
        AssertRectangleBound(upperN, upperD, reverse ? 1 : 5, reverse ? 2 : 6);
        Assert.True(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            a, b, default, lowerN, lowerD, upperN, upperD));
        AssertRectangleBound(lowerN, lowerD, 1, 6); AssertRectangleBound(upperN, upperD, 5, 6);

        // The full original face admits these projections; its retained right half does not.
        a = frame.GetPoint(new(-Fixed64.One - Fixed64.Half, Fixed64.Zero, (Fixed64)3));
        b = frame.GetPoint(new(-Fixed64.Half, Fixed64.Zero, (Fixed64)3));
        Assert.False(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            a, b, cut, lowerN, lowerD, upperN, upperD));
        a = frame.GetPoint(new(Fixed64.FromRaw(-1), Fixed64.Zero, (Fixed64)3));
        Assert.False(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            a, a, cut, lowerN, lowerD, upperN, upperD));
        foreach (Vector3d point in new[] { new Vector3d(0, 1, 3), new(2, 1, 3), new(0, -1, 3) })
        {
            a = frame.GetPoint(point);
            Assert.True(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
                a, a, cut, lowerN, lowerD, upperN, upperD));
        }
        a = frame.GetPoint(new(Fixed64.One, Fixed64.One + Fixed64.MinIncrement, (Fixed64)3));
        Assert.False(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            a, a, cut, lowerN, lowerD, upperN, upperD));
        a = frame.GetPoint(new(Fixed64.One, -Fixed64.One - Fixed64.MinIncrement, (Fixed64)3));
        Assert.False(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            a, a, cut, lowerN, lowerD, upperN, upperD));
        a = frame.GetPoint(new(0, 0, 3)); b = frame.GetPoint(new(-1, 0, 3));
        Assert.True(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            a, b, cut, lowerN, lowerD, upperN, upperD));
        AssertRectangleBound(lowerN, lowerD, 0, 1); AssertRectangleBound(upperN, upperD, 0, 1);
        a = frame.GetPoint(new(-3, -2, 3)); b = frame.GetPoint(new(3, 0, 3));
        Assert.True(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            a, b, cut, lowerN, lowerD, upperN, upperD));
        AssertRectangleBound(lowerN, lowerD, 1, 2); AssertRectangleBound(upperN, upperD, 5, 6);
        a = frame.GetPoint(new(-3, 2, 3)); b = frame.GetPoint(new(0, -2, 3));
        Assert.False(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            a, b, cut, lowerN, lowerD, upperN, upperD));
        // The Y=1 crossing lies beyond the already retained X endpoint;
        // reversing the segment must not loosen either exact X bound.
        a = frame.GetPoint(new((Fixed64)(-3), Fixed64.Zero, (Fixed64)3));
        b = frame.GetPoint(new((Fixed64)3, Fixed64.FromFraction(9, 8), (Fixed64)3));
        Assert.True(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            reverse ? b : a, reverse ? a : b, default, lowerN, lowerD, upperN, upperD));
        AssertRectangleBound(lowerN, lowerD, 1, 6); AssertRectangleBound(upperN, upperD, 5, 6);
        var degenerate = new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        Assert.False(WideTriangleRelations.TryGetProjectedRectangleInterval(degenerate, frame,
            a, a, cut, lowerN, lowerD, upperN, upperD));
    }

    [Fact]
    public void ProjectedRectangle_ShouldRetainAFullWidthCutWhoseMappedDenominatorExceedsSigned832()
    {
        var frame = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        var rectangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Up);
        var n = new Signed832(1UL << 61, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
        var d = new Signed832(1UL << 62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3);
        var cut = new RationalSegmentInterval(n, d, W(1), W(1));
        RationalPoint3d a = frame.GetPoint(new(Fixed64.Quarter, Fixed64.Half, Fixed64.One)),
            b = frame.GetPoint(new(Fixed64.One, Fixed64.Half, Fixed64.One));
        const int words = 32;
        Span<ulong> values = stackalloc ulong[14 * words]; Span<int> signs = stackalloc int[14];
        var lowerN = ContactQuadratic.At(values, signs, 0, words);
        var lowerD = ContactQuadratic.At(values, signs, 1, words);
        var upperN = ContactQuadratic.At(values, signs, 2, words);
        var upperD = ContactQuadratic.At(values, signs, 3, words);
        var expectedN = ContactQuadratic.At(values, signs, 4, words);
        var expectedD = ContactQuadratic.At(values, signs, 5, words);
        var term = ContactQuadratic.At(values, signs, 6, words);
        Assert.True(WideTriangleRelations.TryGetProjectedRectangleInterval(rectangle, frame,
            a, b, cut, lowerN, lowerD, upperN, upperD));
        // u=1/4+3*t/4 gives t=(4*N-D)/(3*D); this fraction is irreducible.
        expectedN.Set(n); expectedN.Add(expectedN); expectedN.Add(expectedN);
        term.Set(d); expectedN.Add(term, -1);
        expectedD.Set(d); expectedD.Add(expectedD); expectedD.Add(term);
        Assert.True((expectedD.Rational[12] & (1UL << 63)) != 0);
        Assert.Equal(0, ContactQuadratic.CompareRatios(lowerN, lowerD, ReadOnlySpan<ulong>.Empty,
            expectedN, expectedD, ReadOnlySpan<ulong>.Empty));
        AssertRectangleBound(upperN, upperD, 1, 1);
    }

    private static void AssertRectangleBound(ContactQuadratic numerator, ContactQuadratic denominator, long n, long d)
    {
        const int words = 32;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var expectedN = ContactQuadratic.At(values, signs, 0, words);
        var expectedD = ContactQuadratic.At(values, signs, 1, words);
        expectedN.Set(W(n)); expectedD.Set(W(d));
        Assert.Equal(0, ContactQuadratic.CompareRatios(numerator, denominator, ReadOnlySpan<ulong>.Empty,
            expectedN, expectedD, ReadOnlySpan<ulong>.Empty));
    }

    [Fact]
    public void PlaneAndRectangleWitnesses_ShouldRejectUndefinedPlanesAndOutsideFiniteCoordinates()
    {
        var frame = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        RationalPoint3d point = frame.GetPoint(new(Fixed64.Half, Fixed64.Half, Fixed64.One));
        Assert.False(WideTriangleRelations.TryGetProjectedPlanePointAnchor(default, frame, point, Vector3d.Zero, out _));
        var rectangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Up);
        const int words = 32;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var n = ContactQuadratic.At(values, signs, 0, words);
        var d = ContactQuadratic.At(values, signs, 1, words);
        n.Clear(); d.Set(W(1));
        Assert.True(WideTriangleRelations.TryGetProjectedRectangleFaceAnchors(rectangle, frame,
            point, point, n, d, Vector3d.Zero, Vector3d.Zero, out var source, out var target));
        Assert.True(source.TryGetPoint(out Vector3d a)); Assert.True(target.TryGetPoint(out Vector3d b));
        Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.One), a);
        Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero), b);
        foreach (Vector3d outside in new[] { new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.One),
            new(Fixed64.Half, -Fixed64.Half, Fixed64.One), new(Fixed64.One + Fixed64.Half, Fixed64.Half, Fixed64.One),
            new(Fixed64.Half, Fixed64.One + Fixed64.Half, Fixed64.One) })
        {
            point = frame.GetPoint(outside);
            Assert.False(WideTriangleRelations.TryGetProjectedRectangleFaceAnchors(rectangle, frame,
                point, point, n, d, Vector3d.Zero, Vector3d.Zero, out _, out _));
        }
    }

    [Fact]
    public void DegenerateFaceAndUnrepresentableTranslatedWitness_ShouldFailExplicitly()
    {
        ScaledRigidFrame3d frame = new(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        RationalPoint3d point = frame.GetPoint(new(1, 1, 1));
        var degenerate = new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        Assert.False(WideTriangleRelations.ContainsProjection(degenerate, frame, point));
        Assert.False(WideTriangleRelations.TryGetProjectedSegmentInterval(degenerate, frame, point, point,
            out _, out _, out _, out _));
        Assert.False(WideTriangleRelations.TryGetProjectedPointAnchor(Face, frame, point,
            new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero), out _));
    }
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public void ScaledConeQuery_ShouldPreserveFiniteFaceEdgeAndMissReduction(int fixture)
    {
        FixedTriangle physical = fixture switch
        {
            0 => new(new(1,-100,-100),new(1,-100,100),new(1,100,0)),
            1 => new(new(-100,103,-100),new(-100,103,100),new(100,-97,0)),
            2 => new(new(2,1,-2),new(2,1,2),new(-2,5,0)),
            3 => new(new(-100,3,-100),new(-100,3,100),new(100,3,0)),
            4 => new(new(10,1,-2),new(10,1,2),new(8,5,0)),
            _ => new(new(1,2,0),new(1,2,0),new(1,2,0))
        };
        FixedTriangle authored = new(physical.A*4,physical.B*4,physical.C*4);
        Vector3d reference = new(7,0,-3);
        var frame = new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,reference,
            Vector3d.One/4,FixedQuaternion.Identity,reference,Vector3d.One);
        bool expected=physical.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero,Vector3d.Up,
            (Fixed64)14,(Fixed64)7,out Vector3d expectedPoint);
        Assert.Equal(expected,WideTriangleConeIntersection.TryGetMinimumAxialPoint(authored,frame,
            Vector3d.Zero,Vector3d.Up,(Fixed64)14,(Fixed64)7,out FixedPointAnchor anchor));
        if(!expected)return;
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(expectedPoint,point);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ScaledPlanarProjection_ShouldRetainClosestFiniteBoundaryAndExactRadiusAdmission(bool outside)
    {
        var triangle=new FixedTriangle(new(0,0,0),new(4,0,0),new(0,0,4));
        var frame=new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            Vector3d.One/4,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        Vector2d center=new(Fixed64.One+Fixed64.One/4,Fixed64.Zero);
        Fixed64 radius=outside?Fixed64.FromRaw(Fixed64.One.m_rawValue/4-1):Fixed64.One/4;
        Assert.Equal(!outside,WidePlanarProjection.TryGetTriangleRelation(center,radius,triangle,frame,out var relation));
        if(outside)return;
        Assert.Equal(Fixed64.One/4,relation.Distance);
        Assert.Equal(new Vector2d(-Fixed64.One/4,Fixed64.Zero),relation.Offset);
    }

    [Fact]
    public void RationalClosestQuery_ShouldCancelItsIndependentRigidDenominatorBeforeChoosingFiniteEdge()
    {
        var triangle=new FixedTriangle(new(0,0,0),new(4,0,0),new(0,0,4));
        var source=new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            Vector3d.One/4,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        Vector3d queryPoint=new(2,1,0);
        var queryFrame=new ScaledRigidFrame3d(queryPoint,
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17,(Fixed64)23,(Fixed64)31),Vector3d.Zero,
            Vector3d.One/4,FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)19,(Fixed64)7,(Fixed64)41),
            Vector3d.Zero,Vector3d.One);
        Assert.True(WideTriangleRelations.TryGetClosestPointAnchor(triangle,source,
            queryFrame.GetPoint(Vector3d.Zero),Vector3d.Zero,out FixedPointAnchor anchor));
        Assert.Equal(Vector3d.Right,anchor.LocalPoint);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ScaledIntersectingSheets_ShouldRetainTriangleContactWithoutRequiringWholeSupportFaces(bool separated)
    {
        var first=new FixedTriangle(new(-4,0,-4),new(4,0,-4),new(0,0,4));
        var second=new FixedTriangle(new(0,-4,-4),new(0,4,-4),new(0,0,4));
        var a=new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            Vector3d.One/4,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        Vector3d offset=separated?Vector3d.Right*3:Vector3d.Zero;
        var b=new ScaledRigidFrame3d(offset,FixedQuaternion.Identity,Vector3d.Zero,
            Vector3d.One/4,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        Assert.Equal(!separated,WideTriangleRelations.TryGetContact(first,a,Vector3d.Zero,
            second,b,offset,out FixedContactAnchors contact));
        if(separated)return;
        Assert.True(contact.FirstAnchor.TryGetPoint(out _));
        Assert.True(contact.SecondAnchor.TryGetPoint(out _));
        Assert.True(contact.Depth>=Fixed64.Zero);
    }

    [Fact]
    public void PointTriangle_ShouldRetainClosestVertexButDeclineTrianglePairAdmission()
    {
        var point=new FixedTriangle(Vector3d.Right,Vector3d.Right,Vector3d.Right);
        var frame=new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            Vector3d.One/4,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        Assert.True(WideTriangleRelations.TryGetClosestPointAnchor(point,frame,Vector3d.Up,Vector3d.Zero,out var closest));
        Assert.Equal(Vector3d.Right/4,closest.LocalPoint);
        Assert.False(WideTriangleRelations.TryGetContact(point,frame,Vector3d.Zero,Face,frame,Vector3d.Zero,out _));
        Assert.False(WideTriangleRelations.TryGetContact(Face,frame,Vector3d.Zero,point,frame,Vector3d.Zero,out _));
    }

    [Theory]
    [InlineData(0,false)]
    [InlineData(0,true)]
    [InlineData(1,false)]
    [InlineData(1,true)]
    [InlineData(2,false)]
    [InlineData(2,true)]
    public void IndependentScaledEdges_ShouldRetainFiniteIncidenceEndpointsAndClippedInterior(int axis,bool reverse)
    {
        Vector3d direction=axis==0?Vector3d.Right:axis==1?Vector3d.Up:Vector3d.Forward;
        ScaledRigidFrame3d sourceFrame=Scaled(Fixed64.Quarter),targetFrame=Scaled(Fixed64.Half);
        var source=new FixedSegment(Vector3d.Zero,direction*4);
        Vector3d first=direction*Fixed64.Half,last=direction*((Fixed64)3/2);
        var target=new FixedSegment(reverse?last:first,reverse?first:last);
        int[] expected=reverse?new[]{-1,2,1,0,-1}:new[]{-1,0,1,2,-1};
        for(int quarter=0;quarter<=4;quarter++)
            Assert.Equal(expected[quarter],WideTriangleRelations.LocateEdgePoint(source,sourceFrame,W(quarter),W(4),target,targetFrame));
        var clip=new RationalSegmentInterval(L(1),L(4),L(3),L(4));
        Assert.Equal(1,WideTriangleRelations.LocateEdgePoint(source,sourceFrame,W(2),W(4),target,targetFrame,clip));
        Assert.Equal(-1,WideTriangleRelations.LocateEdgePoint(source,sourceFrame,W(1),W(4),target,targetFrame,clip));
        var point=new FixedSegment(direction,direction);
        Assert.Equal(0,WideTriangleRelations.LocateEdgePoint(source,sourceFrame,W(2),W(4),point,targetFrame));
        Assert.Equal(-1,WideTriangleRelations.LocateEdgePoint(source,sourceFrame,W(1),W(4),point,targetFrame));
        Vector3d transverse=axis==0?Vector3d.Up:Vector3d.Right;
        var separate=new FixedSegment(first+transverse,last+transverse);
        Assert.Equal(-1,WideTriangleRelations.LocateEdgePoint(source,sourceFrame,W(2),W(4),separate,targetFrame));
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(17, false)]
    [InlineData(-43, false)]
    [InlineData(17, true)]
    [InlineData(-43, true)]
    public void NativeScalarEdges_ShouldMapExactRationalPointsAcrossIndependentForwardCharts(int degrees, bool reverse)
    {
        Fixed64 yaw = Fixed64.FromFraction(degrees, 180) * Fixed64.Pi;
        Fixed64 cosine = FixedMath.Cos(yaw), sine = FixedMath.Sin(yaw);
        Vector3d origin = new(1900000000, -3, -1800000000);
        var native = new NativePrismYawFrame3d(origin, yaw);
        var shiftedNative = new NativePrismYawFrame3d(origin + Vector3d.Up * 2, yaw);
        var world = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        var localSource = new FixedSegment(new(-2, 1, -1), new(6, 1, 3));
        var localTarget = new FixedSegment(new(0, 1, 0), new(4, 1, 2));
        var shiftedTarget = new FixedSegment(new(0, -1, 0), new(4, -1, 2));
        var worldSource = new FixedSegment(ExactPoint(-2, 1, -1), ExactPoint(6, 1, 3));
        var worldTarget = new FixedSegment(ExactPoint(0, 1, 0), ExactPoint(4, 1, 2));
        if (reverse)
        {
            localTarget = new(localTarget.End, localTarget.Start);
            shiftedTarget = new(shiftedTarget.End, shiftedTarget.Start);
            worldTarget = new(worldTarget.End, worldTarget.Start);
        }
        Check(localSource, native, worldTarget, world);
        Check(worldSource, world, localTarget, native);
        Check(localSource, native, shiftedTarget, shiftedNative);
        var authored = new ScaledRigidFrame3d(origin,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)(-11), Fixed64.Zero),
            Vector3d.Zero, Vector3d.One,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)23, Fixed64.Zero),
            Vector3d.Zero, Vector3d.One);
        var verticalSource = new FixedSegment(new(0, -2, 0), new(0, 6, 0));
        var verticalTarget = new FixedSegment(reverse ? new Vector3d(0, 4, 0) : Vector3d.Zero,
            reverse ? Vector3d.Zero : new Vector3d(0, 4, 0));
        // Independent quaternion denominators leave the vertical line exact.
        Check(verticalSource, authored, verticalTarget, native);
        Check(verticalSource, native, verticalTarget, authored);

        // Whole local coordinates make these scalar products exact raw values;
        // this oracle never calls the frame or rounds a transformed point.
        Vector3d ExactPoint(int x, int y, int z) => new(origin.X + cosine * x - sine * z,
            origin.Y + y, origin.Z + sine * x + cosine * z);

        void Check<TSource, TTarget>(FixedSegment source, TSource sourceFrame, FixedSegment target, TTarget targetFrame)
            where TSource : struct, IForwardHullFrame3d where TTarget : struct, IForwardHullFrame3d
        {
            const int words = 32;
            Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
            var n = ContactQuadratic.At(values, signs, 0, words);
            var d = ContactQuadratic.At(values, signs, 1, words);
            // A common factor wider than Signed832 proves the parameter remains borrowed.
            d.Set(W(4)); d.Values[20] = 4;
            int[] expected = reverse ? new[] { -1, 2, 1, 0, -1 } : new[] { -1, 0, 1, 2, -1 };
            for (int quarter = 0; quarter <= 4; quarter++)
            {
                n.Set(W(quarter)); n.Values[20] = (ulong)quarter;
                Assert.Equal(expected[quarter], WideTriangleRelations.LocateFrameEdgePoint(source, sourceFrame,
                    n, d, target, targetFrame));
            }
            n.Set(W(2)); n.Values[20] = 2;
            var clip = new RationalSegmentInterval(L(1), L(4), L(3), L(4));
            Assert.Equal(1, WideTriangleRelations.LocateFrameEdgePoint(source, sourceFrame, n, d, target, targetFrame, clip));
            n.Set(W(1)); n.Values[20] = 1;
            Assert.Equal(-1, WideTriangleRelations.LocateFrameEdgePoint(source, sourceFrame, n, d, target, targetFrame, clip));
        }
    }

    [Theory]
    [InlineData(false,false)]
    [InlineData(true,false)]
    [InlineData(false,true)]
    public void ExactEdgeCoverage_ShouldPreserveFiniteFilledSidesAcrossEquivalentOrDifferentCharts(bool shared,bool folded)
    {
        ScaledRigidFrame3d firstFrame=Scaled(Fixed64.Quarter),secondFrame=Scaled(shared?Fixed64.Quarter:Fixed64.Half);
        var source=new FixedTriangle(new(-4,0,0),new(4,0,0),folded?new Vector3d(0,4,2):new Vector3d(0,4,0));
        var target=shared?new FixedTriangle(new(-2,0,0),new(2,0,0),new(0,-4,0))
            :new FixedTriangle(new(-1,0,0),new(1,0,0),new(0,-2,0));
        Assert.True(WideTriangleRelations.TryGetEdgeCoverage(source,firstFrame,target,secondFrame,
            out var lowerN,out var lowerD,out var upperN,out var upperD,out int sides));
        Assert.Equal(folded?4:2,sides);
        Assert.Equal(0,WideArithmetic.CompareNonNegativeProducts(lowerN,W(4),W(1),lowerD));
        Assert.Equal(0,WideArithmetic.CompareNonNegativeProducts(upperN,W(4),W(3),upperD));
        var away=new FixedTriangle(target.A+Vector3d.Forward,target.B+Vector3d.Forward,target.C+Vector3d.Forward);
        Assert.False(WideTriangleRelations.TryGetEdgeCoverage(source,firstFrame,away,secondFrame,out _,out _,out _,out _,out _));
        var degenerate=new FixedTriangle(target.A,target.B,target.B);
        Assert.False(WideTriangleRelations.TryGetEdgeCoverage(source,firstFrame,degenerate,secondFrame,out _,out _,out _,out _,out _));
    }

    [Fact]
    public void ProjectedFiniteEdgeWitness_ShouldRetainPhysicalBoundaryAdmissionAndFinalTranslatedAnchors()
    {
        var frame=Scaled(Fixed64.Quarter);
        RationalPoint3d first=frame.GetPoint(new(1,4,1)),last=frame.GetPoint(new(2,4,1));
        Assert.False(WideTriangleRelations.TryGetProjectedSegmentFaceAnchors(Face,frame,first,last,W(1),W(2),
            Vector3d.Zero,Vector3d.Zero,7,out _,out _));
        Assert.True(WideTriangleRelations.TryGetProjectedSegmentFaceAnchors(Face,frame,first,last,W(1),W(2),
            Vector3d.Zero,Vector3d.Zero,0,out var source,out var target,requirePhysicalTargetBoundary:false));
        Assert.Equal(new Vector3d((Fixed64)3/8,Fixed64.One,Fixed64.Quarter),source.LocalPoint);
        Assert.Equal(new Vector3d((Fixed64)3/8,Fixed64.Zero,Fixed64.Quarter),target.LocalPoint);
        first=last=frame.GetPoint(new(2,4,2));
        Assert.True(WideTriangleRelations.TryGetProjectedSegmentFaceAnchors(Face,frame,first,last,W(0),W(1),
            Vector3d.Zero,Vector3d.Zero,1,out _,out _,materialize:false));
        Assert.True(WideTriangleRelations.TryGetProjectedSegmentFaceAnchors(Face,frame,first,last,W(0),W(1),
            Vector3d.Zero,Vector3d.Zero,1,out _,out target));
        Assert.Equal(new Vector3d(Fixed64.Half,Fixed64.Zero,Fixed64.Half),target.LocalPoint);
        Vector3d outside=new(Fixed64.MinValue,Fixed64.Zero,Fixed64.Zero);
        Assert.False(WideTriangleRelations.TryGetProjectedSegmentFaceAnchors(Face,frame,first,last,W(0),W(1),
            outside,Vector3d.Zero,1,out _,out _));
        Assert.False(WideTriangleRelations.TryGetProjectedSegmentFaceAnchors(Face,frame,first,last,W(0),W(1),
            Vector3d.Zero,outside,1,out _,out _));
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void NonlatticeSphereSweep_ShouldRetainFiniteFaceEdgeAndVertexRoots(int feature, bool reverse)
    {
        Fixed64 x = Fixed64.MinIncrement;
        Vector3d first = feature == 2 ? new(x, Fixed64.Zero, Fixed64.Zero) : new(x, (Fixed64)(-4), Fixed64.Zero);
        Vector3d second = feature == 2 ? first : new(x, (Fixed64)4, Fixed64.Zero);
        Vector3d third = feature == 0 ? new(x, Fixed64.Zero, (Fixed64)4) : second;
        var triangle = new FixedTriangle(first, second, third);
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new((Fixed64)3 / 2, Fixed64.One, Fixed64.One), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Assert.False(frame.GetPoint(first).TryGetExactLocalPointIn(Vector3d.Zero, FixedQuaternion.Identity, out _));
        Vector3d start = Vector3d.Right * (reverse ? 2 : -2), end = -start;
        if (feature == 0) { start.Z = Fixed64.One; end.Z = Fixed64.One; }
        Assert.True(WideTriangleRelations.TryGetSweptSphereDistance(triangle, frame,
            new FixedSegment(start, end), (Fixed64)4, Fixed64.Half, out Fixed64 distance));
        // Physical x is exactly 1.5 raw units. The retained root is rounded
        // only after adding/subtracting that fraction from 1.5 distance units.
        long expected = Fixed64.FromFraction(3, 2).m_rawValue + (reverse ? -2L : 2L);
        Assert.Equal(expected, distance.m_rawValue);
        Assert.True(WideTriangleRelations.TryGetSweptSphereDistance(triangle, frame,
            new FixedSegment(Vector3d.Zero, Vector3d.Right), Fixed64.One, Fixed64.Half, out distance));
        Assert.Equal(Fixed64.Zero, distance);
        Assert.False(WideTriangleRelations.TryGetSweptSphereDistance(triangle, frame,
            new FixedSegment(start + Vector3d.Up * 10, end + Vector3d.Up * 10),
            (Fixed64)4, Fixed64.Half, out _));
    }

    [Fact]
    public void NonlatticeSphereSweep_ShouldRejectAContactBeyondTheExactClosedChord()
    {
        Vector3d point = new(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero);
        var triangle = new FixedTriangle(point, point, point);
        var frame = Scaled((Fixed64)3 / 2);
        var query = new FixedSegment(-Vector3d.Right, Vector3d.Zero);
        Assert.False(WideTriangleRelations.TryGetSweptSphereDistance(triangle, frame, query,
            Fixed64.One, Fixed64.MinIncrement, out _));
        Assert.True(WideTriangleRelations.TryGetSweptSphereDistance(triangle, frame, query,
            Fixed64.One, Fixed64.MinIncrement * 2, out Fixed64 distance));
        Assert.Equal(Fixed64.One, distance);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ScaledSegmentQuery_ShouldRetainFiniteCrossingCoplanarIntervalAndClosedEndpoints(bool reverse)
    {
        var frame=Scaled(Fixed64.Quarter);
        Vector3d point=new(Fixed64.Quarter,Fixed64.Zero,Fixed64.Quarter);
        var crossing=reverse?new FixedSegment(point-Vector3d.Up,point+Vector3d.Up)
            :new FixedSegment(point+Vector3d.Up,point-Vector3d.Up);
        Assert.True(WideTriangleRelations.TryGetSegmentIntersection(Face,frame,crossing,out var entry,out var exit));
        Assert.Equal(point,entry);Assert.Equal(entry,exit);
        Assert.True(WideTriangleRelations.TryGetSegmentIntersection(Face,frame,
            new FixedSegment(point,point+Vector3d.Up),out entry,out exit));
        Assert.Equal(point,entry);Assert.Equal(entry,exit);
        var coplanar=new FixedSegment(new((Fixed64)(-1),Fixed64.Zero,Fixed64.Quarter),
            new((Fixed64)2,Fixed64.Zero,Fixed64.Quarter));
        if(reverse)coplanar=new FixedSegment(coplanar.End,coplanar.Start);
        Assert.True(WideTriangleRelations.TryGetSegmentIntersection(Face,frame,coplanar,out entry,out exit));
        Vector3d low=new(Fixed64.Zero,Fixed64.Zero,Fixed64.Quarter),high=new((Fixed64)3/4,Fixed64.Zero,Fixed64.Quarter);
        Assert.Equal(reverse?high:low,entry);Assert.Equal(reverse?low:high,exit);
        Assert.False(WideTriangleRelations.TryGetSegmentIntersection(Face,frame,
            new FixedSegment(point+Vector3d.Up,point+Vector3d.Up*2),out _,out _));
        Assert.False(WideTriangleRelations.TryGetSegmentIntersection(Face,frame,
            new FixedSegment(new(2,1,2),new(2,-1,2)),out _,out _));
        Assert.False(WideTriangleRelations.TryGetSegmentIntersection(Face,frame,
            new FixedSegment(new(-1,0,2),new(2,0,2)),out _,out _));
        var collapsed=new FixedTriangle(Vector3d.Zero,Vector3d.Zero,Vector3d.Zero);
        Assert.False(WideTriangleRelations.TryGetSegmentIntersection(collapsed,frame,crossing,out _,out _));
    }

    [Fact]
    public void ScaledClosestAndSupportQueries_ShouldRetainFiniteFeaturesAndSignedPlaneSeparation()
    {
        var frame=Scaled(Fixed64.Quarter);
        WideTriangleRelations.GetScaledAreaAndNormal(Face,frame,out Fixed64 area,out Vector3d normal);
        Assert.Equal(Fixed64.Half,area);Assert.Equal(-Vector3d.Up,normal);
        foreach(var fixture in new[]{
            (new Vector3d(-1,1,-1),Vector3d.Zero),
            (new Vector3d(2,1,0),Vector3d.Right),
            (new Vector3d(Fixed64.Quarter,Fixed64.One,Fixed64.Quarter),new Vector3d(Fixed64.Quarter,Fixed64.Zero,Fixed64.Quarter)),
            (new Vector3d(Fixed64.One,Fixed64.One,Fixed64.One),new Vector3d(Fixed64.Half,Fixed64.Zero,Fixed64.Half))})
        {
            Assert.True(WideTriangleRelations.TryGetClosestPointAnchor(Face,frame,fixture.Item1,Vector3d.Zero,out var closest));
            Assert.Equal(fixture.Item2,closest.LocalPoint);
        }
        var support=new FixedPointAnchor(Vector3d.Zero,FixedQuaternion.Identity,
            new(Fixed64.Quarter,-Fixed64.Quarter,Fixed64.Quarter));
        Assert.True(WideTriangleRelations.TryGetProjectedSupportContact(Face,frame,support,Vector3d.Up,Vector3d.Zero,out var contact));
        Assert.Equal(Fixed64.Quarter,contact.Depth);Assert.Equal(Vector3d.Up,contact.Normal);
        Assert.Equal(new Vector3d(Fixed64.Quarter,Fixed64.Zero,Fixed64.Quarter),contact.FirstAnchor.LocalPoint);
        Assert.False(WideTriangleRelations.TryGetProjectedSupportContact(Face,frame,support,Vector3d.Right,Vector3d.Zero,out _));
        Assert.False(WideTriangleRelations.TryGetProjectedSupportContact(Face,frame,support,-Vector3d.Up,Vector3d.Zero,out _));
        support=new FixedPointAnchor(Vector3d.Zero,FixedQuaternion.Identity,new(2,-1,2));
        Assert.False(WideTriangleRelations.TryGetProjectedSupportContact(Face,frame,support,Vector3d.Up,Vector3d.Zero,out _));
        Vector3d outsideOrigin=new(Fixed64.MinValue,Fixed64.Zero,Fixed64.Zero);
        Assert.False(WideTriangleRelations.TryGetClosestPointAnchor(Face,frame,new(-1,1,-1),outsideOrigin,out _));
        var collapsed=new FixedTriangle(Vector3d.Zero,Vector3d.Zero,Vector3d.Zero);
        Assert.False(WideTriangleRelations.TryGetClosestPointAnchor(collapsed,frame,Vector3d.Up,outsideOrigin,out _));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public void FiniteNonparallelSegmentFeet_ShouldRejectOnlyOutOfRangeLineParameters(int fixture)
    {
        var first=new FixedSegment(Vector3d.Zero,Vector3d.Right);
        var second=fixture switch
        {
            0=>new FixedSegment(new(Fixed64.Half,-Fixed64.One,Fixed64.Zero),new(Fixed64.Half,Fixed64.One,Fixed64.Zero)),
            1=>new FixedSegment(new(2,-1,0),new(2,1,0)),
            2=>new FixedSegment(new(-1,-1,0),new(-1,1,0)),
            3=>new FixedSegment(new(Fixed64.Half,Fixed64.One,Fixed64.Zero),new(Fixed64.Half,(Fixed64)2,Fixed64.Zero)),
            4=>new FixedSegment(new(Fixed64.Half,(Fixed64)(-2),Fixed64.Zero),new(Fixed64.Half,-Fixed64.One,Fixed64.Zero)),
            _=>new FixedSegment(Vector3d.Up,Vector3d.Up+Vector3d.Right)
        };
        const int words=27;
        System.Span<ulong> values=stackalloc ulong[10*words];System.Span<int> signs=stackalloc int[10];
        var an=ContactQuadratic.At(values,signs,0,words);var ad=ContactQuadratic.At(values,signs,1,words);
        var bn=ContactQuadratic.At(values,signs,2,words);var bd=ContactQuadratic.At(values,signs,3,words);
        bool admitted=WideTriangleRelations.TryGetClosestNonparallelSegmentParameters(
            RationalPoint3d.FromScalarPoint(first.Start),RationalPoint3d.FromScalarPoint(first.End),
            RationalPoint3d.FromScalarPoint(second.Start),RationalPoint3d.FromScalarPoint(second.End),an,ad,bn,bd);
        Assert.Equal(fixture==0,admitted);
        if(admitted)
        {
            var difference=ContactQuadratic.At(values,signs,4,words);
            ContactQuadratic.Scale(an,Signed576.ExtendValue(Signed192.Signed(2)),difference);difference.Add(ad,-1);
            Assert.Equal(0,difference.Signs[0]);
            ContactQuadratic.Scale(bn,Signed576.ExtendValue(Signed192.Signed(2)),difference);difference.Add(bd,-1);
            Assert.Equal(0,difference.Signs[0]);
        }
    }

    [Theory]
    [InlineData(1, 2, 0)]
    [InlineData(3, 2, 2)]
    [InlineData(5, 2, 2)]
    [InlineData(7, 2, 4)]
    [InlineData(1, 4, 1)]
    [InlineData(3, 4, 2)]
    public void NonlatticePlanarDistance_ShouldRoundAtTheRawMidpointAfterExactRadiusAdmission(long raw,int divisor,long expected)
    {
        Fixed64 scale=divisor==2?Fixed64.Half:(Fixed64)3/4;
        var frame=Scaled(scale);Vector3d point=new(Fixed64.FromRaw(raw),Fixed64.Zero,Fixed64.Zero);
        var triangle=new FixedTriangle(point,point,point);
        Assert.True(WidePlanarProjection.TryGetTriangleRelation(Vector2d.Zero,Fixed64.One,triangle,frame,out var relation));
        Assert.Equal(expected,relation.Distance.m_rawValue);Assert.Equal(expected,relation.Offset.X.m_rawValue);
        Assert.False(relation.IsContained);Assert.Equal(Vector2d.Right,relation.Direction);
        Assert.False(WidePlanarProjection.TryGetTriangleRelation(Vector2d.Zero,Fixed64.Zero,triangle,frame,out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ScaledPlanarRelation_ShouldRetainFiniteEdgeFeetAndFilledInteriorAcrossWinding(bool reverse)
    {
        var triangle=reverse?new FixedTriangle(Face.C,Face.B,Face.A):Face;
        var frame=Scaled(Fixed64.Quarter);
        Assert.True(WidePlanarProjection.TryGetTriangleRelation(new(Fixed64.Half,-Fixed64.Quarter),
            Fixed64.Quarter,triangle,frame,out var edge));
        Assert.Equal(Fixed64.Quarter,edge.Distance);Assert.Equal(new Vector2d(Fixed64.Zero,Fixed64.Quarter),edge.Offset);
        Assert.True(WidePlanarProjection.TryGetTriangleRelation(new(Fixed64.Quarter,Fixed64.Quarter),
            Fixed64.Zero,triangle,frame,out var interior));Assert.True(interior.IsContained);
        var line=new FixedTriangle(Vector3d.Zero,Vector3d.Right*4,Vector3d.Right*4);
        Assert.True(WidePlanarProjection.TryGetTriangleRelation(new(Fixed64.Half,Fixed64.Zero),
            Fixed64.Zero,line,frame,out var onLine));Assert.True(onLine.IsContained);
        var rigid=ScaledRigidFrame3d.Rigid(Vector3d.Zero,FixedQuaternion.Identity);
        Assert.True(WidePlanarProjection.TryGetTriangleRelation(new((Fixed64)5,Fixed64.Zero),
            Fixed64.One,triangle,rigid,out var rigidEdge));Assert.Equal(Fixed64.One,rigidEdge.Distance);
    }

    [Fact]
    public void ScaledTriangleContactAndProjectedWitness_ShouldDeclineDegenerateOutsideOrUnrepresentableFiniteFeatures()
    {
        var frame=Scaled(Fixed64.Quarter);var rigid=ScaledRigidFrame3d.Rigid(Vector3d.Zero,FixedQuaternion.Identity);
        var crossing=new FixedTriangle(new(0,-4,-4),new(0,4,-4),new(0,0,4));
        Assert.True(WideTriangleRelations.TryGetContact(Face,rigid,Vector3d.Zero,
            crossing,rigid,Vector3d.Zero,out _));
        var collapsed=new FixedTriangle(Vector3d.Zero,Vector3d.Zero,Vector3d.Zero);
        Assert.False(WideTriangleRelations.TryGetContact(Face,frame,Vector3d.Zero,
            collapsed,frame,Vector3d.Zero,out _));
        Vector3d far=new(Fixed64.MinValue,Fixed64.Zero,Fixed64.Zero);
        Assert.False(WideTriangleRelations.TryGetContact(Face,frame,far,crossing,frame,Vector3d.Zero,out _));
        Assert.False(WideTriangleRelations.TryGetContact(Face,frame,Vector3d.Zero,crossing,frame,far,out _));
        RationalPoint3d outside=frame.GetPoint(new(-2,4,-2));
        Assert.False(WideTriangleRelations.TryGetProjectedSegmentFaceAnchors(Face,frame,outside,outside,
            W(0),W(1),Vector3d.Zero,Vector3d.Zero,0,out _,out _,requirePhysicalTargetBoundary:false));
        Assert.False(WideTriangleRelations.TryGetProjectedSegmentFaceAnchors(collapsed,frame,outside,outside,
            W(0),W(1),Vector3d.Zero,Vector3d.Zero,0,out _,out _,requirePhysicalTargetBoundary:false));
        var source=new FixedTriangle(Vector3d.Zero,Vector3d.Right*4,Vector3d.Forward*4);
        var remote=new FixedTriangle(new(100,0,100),new(104,0,100),new(100,0,104));
        Assert.False(WideTriangleRelations.TryGetEdgeCoverage(source,frame,remote,Scaled(Fixed64.Half),
            out _,out _,out _,out _,out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RigidSourceConeIngress_ShouldRetainItsFiniteMinimumOrRejectTheMiss(bool separated)
    {
        Fixed64 x=separated?(Fixed64)10:Fixed64.One;
        var triangle=new FixedTriangle(new(x,(Fixed64)(-100),(Fixed64)(-100)),
            new(x,(Fixed64)(-100),(Fixed64)100),new(x,(Fixed64)100,Fixed64.Zero));
        Assert.Equal(!separated,WideTriangleConeIntersection.TryGetMinimumAxialPoint(triangle,
            ScaledRigidFrame3d.Rigid(Vector3d.Zero,FixedQuaternion.Identity),Vector3d.Zero,Vector3d.Up,
            (Fixed64)14,(Fixed64)7,out FixedPointAnchor anchor));
        if(separated)return;
        Assert.True(anchor.TryGetPoint(out Vector3d point));Assert.Equal(new Vector3d(1,2,0),point);
    }

    [Fact]
    public void AffineEdgeCoverage_ShouldRetainTheWholeEdgeAndBothFilledSidesWithoutAdmittingDegenerateSources()
    {
        var frame=Scaled(Fixed64.Quarter);
        var source=new FixedTriangle(new(-1,0,0),new(1,0,0),new(0,1,0));
        var below=new FixedTriangle(new(-4,0,0),new(4,0,0),new(0,-4,0));
        Assert.True(WideTriangleRelations.TryGetEdgeCoverage(source,frame,below,frame,
            out var lowerN,out var lowerD,out var upperN,out var upperD,out int sides));
        Assert.Equal(2,sides);Assert.True(lowerN.IsZero);
        Assert.Equal(0,WideArithmetic.CompareNonNegativeProducts(upperN,W(1),W(1),upperD));
        var above=new FixedTriangle(new(-2,0,0),new(2,0,0),new(0,2,0));
        Assert.True(WideTriangleRelations.TryGetEdgeCoverage(source,frame,above,Scaled(Fixed64.Half),
            out _,out _,out _,out _,out sides));Assert.Equal(1,sides);
        var point=new FixedTriangle(Vector3d.Zero,Vector3d.Zero,Vector3d.Zero);
        Assert.False(WideTriangleRelations.TryGetEdgeCoverage(point,frame,above,Scaled(Fixed64.Half),
            out _,out _,out _,out _,out _));
    }

    [Fact]
    public void ProjectedFiniteSegment_ShouldKeepTheStrictestAlreadyRetainedEntryAndExitHalfplanes()
    {
        var frame=Scaled(Fixed64.Quarter);
        Assert.True(WideTriangleRelations.TryGetProjectedSegmentInterval(Face,frame,
            frame.GetPoint(new(-1,1,-2)),frame.GetPoint(new(2,1,2)),out var ln,out var ld,out _,out _));
        Assert.Equal(0,WideArithmetic.CompareNonNegativeProducts(ln,W(2),W(1),ld));
        Assert.True(WideTriangleRelations.TryGetProjectedSegmentInterval(Face,frame,
            frame.GetPoint(new(1,1,1)),frame.GetPoint(new(-1,1,-3)),out _,out _,out var un,out var ud));
        Assert.Equal(0,WideArithmetic.CompareNonNegativeProducts(un,W(4),W(1),ud));
    }


    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    [InlineData(9)]
    public void ScaledConeMinimum_RetainsFiniteBoundaryRootCellsAndWinding(int fixture)
    {
        Fixed64 height = fixture < 3 ? Fixed64.MaxValue : (Fixed64)6;
        Fixed64 radius = fixture == 1 ? Fixed64.Two : (Fixed64)3;
        long floor = fixture == 1 ? long.MaxValue >> 1 : long.MaxValue / 3;
        Fixed64 y = Fixed64.FromRaw(floor);
        FixedTriangle physical = fixture switch
        {
            0 or 1 => new(new(Fixed64.One,y,-Fixed64.One),
                new(Fixed64.One,Fixed64.FromRaw(floor+3),(Fixed64)3),new(Fixed64.One,y+Fixed64.One,Fixed64.Zero)),
            2 => new(new(Fixed64.One,y,-Fixed64.One),
                new(Fixed64.One,Fixed64.FromRaw(floor+3),(Fixed64)3),new(Fixed64.One,y-Fixed64.One,Fixed64.Zero)),
            3 => new(new(-2,3,-2),new(2,3,-2),new(0,3,2)),
            4 => new(new(-100,-1,-100),new(100,-1,-100),new(0,-1,100)),
            5 => new(new(-100,7,-100),new(100,7,-100),new(0,7,100)),
            6 => new(new(10,3,-1),new(12,3,-1),new(11,3,1)),
            7 => new(new(10,-100,-100),new(10,-100,100),new(10,100,0)),
            8 => new(new(1,0,0),new(1,6,0),new(1,3,10)),
            _ => new(new(-100,3,-100),new(100,3,-100),new(0,3,100))
        };
        var frame = new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            new(Fixed64.Half,Fixed64.One,Fixed64.Half),FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        // Only X/Z are expanded in the source, retaining exact full-domain Y
        // root cells while using a genuinely scaled ingress.
        Vector3d Author(Vector3d v) => new(v.X*2,v.Y,v.Z*2);
        for(int reverse=0;reverse<2;reverse++)
        {
            if(reverse!=0) physical=new(physical.A,physical.C,physical.B);
            FixedTriangle authored=new(Author(physical.A),Author(physical.B),Author(physical.C));
            bool expected=physical.TryGetFiniteConeIntersectionMinimumAxialPoint(Vector3d.Zero,Vector3d.Up,height,radius,out Vector3d baseline);
            Assert.Equal(expected,WideTriangleConeIntersection.TryGetMinimumAxialPoint(authored,frame,
                Vector3d.Zero,Vector3d.Up,height,radius,out FixedPointAnchor anchor));
            if(!expected)continue;
            Assert.True(anchor.TryGetPoint(out Vector3d point));Assert.Equal(baseline,point);
        }
    }


    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ScaledConeMinimum_UnrepresentablePhysicalLocalPointDeclinesWithoutSaturation(bool axial)
    {
        // These general math inputs exceed collider admission. The source is
        // representable, but its scaled physical X is beyond the local scalar
        // chart, so a finite witness must be declined rather than saturated.
        Fixed64 x = axial ? Fixed64.FromRaw((1L<<62)+(1L<<29)) : (Fixed64)1125000000;
        Fixed64 extent = axial ? (Fixed64)4 : Fixed64.MaxValue;
        FixedTriangle triangle = new(new(x,-extent,-extent),new(x,-extent,extent),new(x,extent,Fixed64.Zero));
        var frame = Scaled(Fixed64.Two);
        Assert.False(frame.GetPoint(triangle.A).TryGetPoint(out _));
        Vector3d axis = axial ? new(Fixed64.One+Fixed64.MinIncrement,Fixed64.Zero,Fixed64.Zero)
            : new Vector3d(1,1,0).Normalized;
        Assert.True(axis.IsNormalized());
        Assert.False(WideTriangleConeIntersection.TryGetMinimumAxialPoint(triangle,frame,Vector3d.Zero,
            axis,Fixed64.MaxValue,axial?Fixed64.One:Fixed64.MaxValue,out _));
    }

    private static ScaledRigidFrame3d Scaled(Fixed64 scale)=>new(Vector3d.Zero,FixedQuaternion.Identity,
        Vector3d.Zero,Vector3d.One*scale,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
    private static Signed832 W(long value)=>Signed832.ExtendValue(Signed192.Signed(value));
    private static Signed320 L(long value)=>Signed320.ExtendValue(Signed192.Signed(value));

}
