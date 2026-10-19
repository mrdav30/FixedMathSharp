using FixedMathSharp.Geometry;
using Xunit;
using System;
using System.Collections.Generic;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class WideOrientedBoxScaledHullTests
{
    private static readonly Vector3d[] Cube = { new(-1, -1, -1), new(1, -1, -1), new(1, 1, -1), new(-1, 1, -1),
        new(-1, -1, 1), new(1, -1, 1), new(1, 1, 1), new(-1, 1, 1) };
    private static readonly int[] Faces = { 0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
        3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5 };
    private static readonly int[] Edges = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4,
        0, 4, 1, 5, 2, 6, 3, 7 };

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(3, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(3, true)]
    public void ExactScaledHull_ShouldPreserveRigidSatClassificationAndDepth(int separation, bool deduplicate)
    {
        FixedQuaternion firstRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)31);
        FixedQuaternion secondRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)(-11), (Fixed64)7, (Fixed64)19);
        Vector3d firstOrigin = new(-2, 1, 3), secondOrigin = firstOrigin + Vector3d.Right * separation;
        ScaledRigidFrame3d first = Frame(firstOrigin, firstRotation), second = Frame(secondOrigin, secondRotation);
        bool expected = WideOrientedBox.TryGetConvexHullPenetration(firstOrigin, firstRotation, Cube, Faces, Edges,
            secondOrigin, secondRotation, Cube, Faces, Edges, out var rigid, deduplicate);
        Assert.Equal(expected, WideOrientedBox.TryGetConvexHullPenetration(first, Cube, Faces, Edges,
            second, Cube, Faces, Edges, out var scaled, deduplicate));
        Assert.Equal(rigid.HasValue, scaled.HasValue);
        if (!expected) return;
        Assert.Equal(rigid.GetRoundedDepth(out bool rigidClamped), scaled.GetRoundedDepth(out bool scaledClamped));
        Assert.Equal(rigidClamped, scaledClamped);
        WideAxis3 oriented = rigid.Negate ? -rigid.Axis : rigid.Axis;
        Assert.Equal(WideNormalization.GetNormalized(Signed576.ExtendValue(oriented.X),
            Signed576.ExtendValue(oriented.Y), Signed576.ExtendValue(oriented.Z)), scaled.GetNormal());
    }

    [Fact]
    public void SignedAdmission_ShouldKeepBothExactExitsAndLeaveOverlapIndependentOfAdmission()
    {
        ScaledRigidFrame3d first = Frame(Vector3d.Zero, FixedQuaternion.Identity);
        ScaledRigidFrame3d second = Frame(Vector3d.Right, FixedQuaternion.Identity);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(first, Cube, Faces, Edges,
            second, Cube, Faces, Edges, out var rejected, true, Reject));
        Assert.False(rejected.HasValue);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(first, Cube, Faces, Edges,
            second, Cube, Faces, Edges, out var admitted, true, AdmitNegativeX));
        Assert.Equal(-Vector3d.Right, admitted.GetNormal());
        Assert.Equal((Fixed64)3, admitted.GetRoundedDepth(out bool clamped));
        Assert.False(clamped);
        static bool Reject(in WideScaledHullPenetration _) => false;
        static bool AdmitNegativeX(in WideScaledHullPenetration value) => value.Axis.Y.IsZero
            && value.Axis.Z.IsZero && value.Axis.X.Sign * (value.Negate ? -1 : 1) < 0;
    }

    [Theory]
    [InlineData(0, true)]
    [InlineData(3, false)]
    public void FractionallyScaledPlanarSheets_ShouldRetainTheirFiniteInPlaneSeparators(int separation, bool overlap)
    {
        Vector3d[] quad = { new(-1, 0, -1), new(1, 0, -1), new(1, 0, 1), new(-1, 0, 1) };
        int[] triangles = { 0, 1, 2, 0, 2, 3 }, edges = { 0, 1, 1, 2, 2, 3, 3, 0 };
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)45);
        Vector3d scale = new((Fixed64)2 / 3, Fixed64.One, (Fixed64)3 / 5);
        ScaledRigidFrame3d first = new(Vector3d.Zero, rotation, Vector3d.Zero, scale, rotation, Vector3d.Zero, Vector3d.One);
        ScaledRigidFrame3d second = new(Vector3d.Right * separation, rotation, Vector3d.Zero, scale,
            rotation, Vector3d.Zero, Vector3d.One);
        Assert.Equal(overlap, WideOrientedBox.TryGetConvexHullPenetration(first, quad, triangles, edges,
            second, quad, triangles, edges, out var result, true, planarPair: true));
        if (overlap) Assert.Equal(Fixed64.Zero, result.GetRoundedDepth(out _));
    }

    [Theory]
    [InlineData(0, 0)]
    [InlineData(2, 0)]
    [InlineData(3, 0)]
    [InlineData(4, 0)]
    [InlineData(5, 0)]
    [InlineData(0, 2)]
    [InlineData(3, 2)]
    [InlineData(4, 2)]
    [InlineData(5, 2)]
    public void ScaledHullCapsule_ShouldRetainRigidReferenceAdmissionDepthAndStrictOverlap(int offset, int length)
    {
        Vector3d[] authored = new Vector3d[Cube.Length], physical = new Vector3d[Cube.Length];
        for (int index = 0; index < Cube.Length; index++)
        { authored[index] = Cube[index] * 4; physical[index] = Cube[index] * 3; }
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * ((Fixed64)3 / 4), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var center = new Vector3d(offset, 0, 0);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)31);
        bool expected = WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(Vector3d.Zero,
            FixedQuaternion.Identity, physical, Faces, Edges, center, rotation, Vector3d.Up,
            (Fixed64)length, Fixed64.One, out var reference);
        Assert.Equal(expected, WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(frame, Vector3d.Zero,
            authored, Faces, Edges, center, rotation, Vector3d.Up, (Fixed64)length, Fixed64.One, out var contact));
        Assert.Equal(WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(Vector3d.Zero,
            FixedQuaternion.Identity, physical, Faces, Edges, center, rotation, Vector3d.Up,
            (Fixed64)length, Fixed64.One), WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(frame,
            authored, Faces, Edges, center, rotation, Vector3d.Up, (Fixed64)length, Fixed64.One));
        if (!expected) return;
        Assert.Equal(reference.Depth, contact.Depth);
        Assert.Equal(reference.Normal, contact.Normal);
        Assert.Equal(reference.DepthIsClamped, contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out var hullPoint));
        Assert.True(contact.SecondAnchor.TryGetPoint(out _));
        Assert.InRange(hullPoint.X.m_rawValue, ((Fixed64)(-3)).m_rawValue, ((Fixed64)3).m_rawValue);
        Assert.InRange(hullPoint.Y.m_rawValue, ((Fixed64)(-3)).m_rawValue, ((Fixed64)3).m_rawValue);
        Assert.InRange(hullPoint.Z.m_rawValue, ((Fixed64)(-3)).m_rawValue, ((Fixed64)3).m_rawValue);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactHullFiniteManifold_ShouldRetainFiniteFaceVerticesOrCrossedEdgeFeet(bool crossed)
    {
        Vector3d[] first=Array.ConvertAll(Cube,p=>new Vector3d(p.X,p.Y*(crossed?2:1),p.Z*(crossed?Fixed64.Quarter:Fixed64.One))*4);
        Vector3d[] second=Array.ConvertAll(Cube,p=>new Vector3d(p.X,p.Y*(crossed?Fixed64.Quarter:Fixed64.Two),p.Z*2)*4);
        Vector3d offset=Vector3d.Right*((Fixed64)3/2);
        ScaledRigidFrame3d a=ScaledFrame(Vector3d.Zero),b=ScaledFrame(offset);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(a,first,Faces,Edges,b,second,Faces,Edges,out var penetration));
        var contacts=new List<FixedContactAnchors>();
        Assert.True(WideOrientedBox.VisitConvexHullContacts(a,Vector3d.Zero,first,Faces,Edges,b,offset,second,Faces,Edges,
            penetration,contacts,static(pool,contact)=>pool.Add(contact))>0);
        foreach(var contact in contacts)
        {
            Assert.Equal(Vector3d.Right,contact.Normal);Assert.Equal(Fixed64.Half,contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));Assert.Equal(Fixed64.One,point.X);
            Assert.Equal(crossed?Fixed64.Quarter:Fixed64.One,point.Y.Abs());
            Assert.Equal(crossed?Fixed64.Quarter:Fixed64.One,point.Z.Abs());
        }
        Assert.Equal(1,WideOrientedBox.VisitConvexHullContacts(a,Vector3d.Zero,first,Faces,Edges,b,offset,second,Faces,Edges,
            penetration,0,static(_,_)=>throw new InvalidOperationException("Admission must retain the witness without materializing it."),admissionOnly:true));
        if(!crossed)
        {
            Assert.True(WideOrientedBox.TryGetConvexHullPenetration(b,second,Faces,Edges,a,first,Faces,Edges,out var reversed));
            contacts.Clear();
            Assert.True(WideOrientedBox.VisitConvexHullContacts(b,offset,second,Faces,Edges,a,Vector3d.Zero,first,Faces,Edges,
                reversed,contacts,static(pool,contact)=>pool.Add(contact))>0);
            foreach(var contact in contacts)
            {
                Assert.Equal(Vector3d.Left,contact.Normal);Assert.Equal(Fixed64.Half,contact.Depth);
                Assert.True(contact.FirstAnchor.TryGetPoint(out var point));Assert.Equal(Fixed64.Half,point.X);
                Assert.True(contact.SecondAnchor.TryGetPoint(out point));Assert.Equal(Fixed64.One,point.X);
            }
        }
        foreach(bool empty in new[]{false,true})
        foreach(bool admissionOnly in new[]{false,true})
        {
            var domain=new RetainedSupportDomain(empty,false);
            Assert.Equal(0,WideOrientedBox.VisitConvexHullContacts(a,Vector3d.Zero,first,Faces,Edges,b,offset,second,Faces,Edges,
                penetration,0,static(_,_)=>throw new InvalidOperationException("An excluded support cannot emit a contact."),domain,admissionOnly));
        }
    }

    [Fact]
    public void ExactHullRetainedEdges_ShouldMaterializeTheirAdmittedInteriorCuts()
    {
        Vector3d[] first=Array.ConvertAll(Cube,p=>p*4),second=Array.ConvertAll(Cube,p=>new Vector3d(p.X,p.Y*2,p.Z*2)*4);
        Vector3d offset=Vector3d.Right*((Fixed64)3/2);ScaledRigidFrame3d a=ScaledFrame(Vector3d.Zero),b=ScaledFrame(offset);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(a,first,Faces,Edges,b,second,Faces,Edges,out var penetration));
        var contacts=new List<FixedContactAnchors>();var domain=new RetainedSupportDomain(false,true);
        Assert.True(WideOrientedBox.VisitConvexHullContacts(a,Vector3d.Zero,first,Faces,Edges,b,offset,second,Faces,Edges,
            penetration,contacts,static(pool,contact)=>pool.Add(contact),domain)>=2);
        foreach(var contact in contacts)
        {
            Assert.Equal(Fixed64.Half,contact.Depth);Assert.Equal(Vector3d.Right,contact.Normal);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d firstPoint));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d secondPoint));
            Assert.Equal(Fixed64.One,firstPoint.X);Assert.Equal(Fixed64.Half,secondPoint.X);
            Assert.Equal(firstPoint.Y,secondPoint.Y);Assert.Equal(firstPoint.Z,secondPoint.Z);
            Assert.True(firstPoint.Y.Abs()==Fixed64.Half||firstPoint.Z.Abs()==Fixed64.Half);
        }
    }

    [Fact]
    public void ExactNonparallelSupportEdges_ShouldRetainOrDeclineTheirRationalIntersection()
    {
        Vector3d[] first={new(-4,0,0),new(4,0,0),new(0,4,-4)};
        Vector3d[] second={new(0,-4,0),new(0,4,0),new(4,0,4)};
        int[] faces={0,1,2},edges={0,1,1,2,2,0};var frame=ScaledFrame(Vector3d.Zero);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(frame,first,faces,edges,frame,second,faces,edges,
            out var penetration,admission:static(in WideScaledHullPenetration value)=>
            {
                WideDirection3d normal=value.Negate?-value.Axis:value.Axis;
                return normal.X.IsZero&&normal.Y.IsZero&&normal.Z.Sign>0;
            }));
        var contacts=new List<FixedContactAnchors>();
        Assert.Equal(1,WideOrientedBox.VisitConvexHullContacts(frame,Vector3d.Zero,first,faces,edges,frame,Vector3d.Zero,
            second,faces,edges,penetration,contacts,static(pool,contact)=>pool.Add(contact)));
        var contact=Assert.Single(contacts);Assert.Equal(Fixed64.Zero,contact.Depth);Assert.Equal(Vector3d.Forward,contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out var a));Assert.Equal(Vector3d.Zero,a);
        Assert.True(contact.SecondAnchor.TryGetPoint(out var b));Assert.Equal(a,b);
        foreach(bool admissionOnly in new[]{false,true})
        foreach(bool acceptFirst in new[]{false,true})
            Assert.Equal(0,WideOrientedBox.VisitConvexHullContacts(frame,Vector3d.Zero,first,faces,edges,frame,Vector3d.Zero,
                second,faces,edges,penetration,0,static(_,_)=>throw new InvalidOperationException("Excluded edge feet cannot emit."),
                new RetainedSupportDomain(false,false,acceptFirst),admissionOnly));
        Assert.Equal(1,WideOrientedBox.VisitConvexHullContacts(frame,Vector3d.Zero,first,faces,edges,frame,Vector3d.Zero,
            second,faces,edges,penetration,0,static(_,_)=>throw new InvalidOperationException("Admission must not materialize."),admissionOnly:true));
        Vector3d far=new(Fixed64.MinValue,Fixed64.Zero,Fixed64.Zero);
        Assert.Equal(0,WideOrientedBox.VisitConvexHullContacts(frame,far,first,faces,edges,frame,Vector3d.Zero,
            second,faces,edges,penetration,0,static(_,_)=>throw new InvalidOperationException("Unrepresentable source feet cannot emit.")));
        Assert.Equal(0,WideOrientedBox.VisitConvexHullContacts(frame,Vector3d.Zero,first,faces,edges,frame,far,
            second,faces,edges,penetration,0,static(_,_)=>throw new InvalidOperationException("Unrepresentable target feet cannot emit.")));
    }

    [Fact]
    public void ScaledBoxHullContact_ShouldRetainSupportVerticesSeparationAndRigidFastPath()
    {
        Vector3d[] authored=Array.ConvertAll(Cube,p=>p*4);var frame=ScaledFrame(Vector3d.Zero);
        Assert.True(WideOrientedBox.TryGetConvexHullContact(Vector3d.Right*((Fixed64)3/2),FixedQuaternion.Identity,
            Vector3d.One,frame,Vector3d.Zero,authored,Faces,Edges,out var contact));
        Assert.Equal(Vector3d.Left,contact.Normal);Assert.Equal(Fixed64.Half,contact.Depth);
        Assert.False(WideOrientedBox.TryGetConvexHullContact(Vector3d.Right*4,FixedQuaternion.Identity,
            Vector3d.One,frame,Vector3d.Zero,authored,Faces,Edges,out _));
        var rigid=Frame(Vector3d.Zero,FixedQuaternion.Identity);
        Assert.True(WideOrientedBox.TryGetConvexHullContact(Vector3d.Right,FixedQuaternion.Identity,Vector3d.One,
            rigid,Vector3d.Zero,Cube,Faces,Edges,out _));
        Vector3d[] collapsed=new Vector3d[Cube.Length];
        Assert.False(WideOrientedBox.TryGetConvexHullPenetration(frame,collapsed,Faces,Edges,frame,collapsed,Faces,Edges,out _));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ScaledHullCapsule_ParallelCoreBeyondFaceMidpoint_ShouldMatchTheFiniteSupportSet(int axis)
    {
        Vector3d[] authored=Array.ConvertAll(Cube,p=>p*4),physical=Array.ConvertAll(Cube,p=>p*3);
        Vector3d normal=axis==0?Vector3d.Right:axis==1?Vector3d.Up:Vector3d.Forward;
        Vector3d coreAxis=axis==0?Vector3d.Up:axis==1?Vector3d.Forward:Vector3d.Right;
        Vector3d center=normal*((Fixed64)7/2)+coreAxis*4;
        var frame=new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            Vector3d.One*((Fixed64)3/4),FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(frame,Vector3d.Zero,authored,Faces,Edges,
            center,FixedQuaternion.Identity,coreAxis,(Fixed64)6,Fixed64.One,out var contact));
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(Vector3d.Zero,FixedQuaternion.Identity,
            physical,Faces,Edges,center,FixedQuaternion.Identity,coreAxis,(Fixed64)6,Fixed64.One,out var reference));
        Assert.Equal(reference.Normal,contact.Normal);Assert.Equal(reference.Depth,contact.Depth);
        Assert.Equal(normal,contact.Normal);Assert.Equal(Fixed64.Half,contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out var hull));Assert.True(contact.SecondAnchor.TryGetPoint(out var capsule));
        Assert.Equal((Fixed64)3,Vector3d.Dot(hull,normal));
        Assert.Equal((Fixed64)5/2,Vector3d.Dot(capsule,normal));
        Fixed64 coordinate=Vector3d.Dot(hull,coreAxis);
        Assert.InRange(coordinate.m_rawValue,Fixed64.One.m_rawValue,((Fixed64)3).m_rawValue);
        Assert.Equal(coordinate,Vector3d.Dot(capsule,coreAxis));
        var rigid=Frame(Vector3d.Zero,FixedQuaternion.Identity);
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(rigid,Vector3d.Zero,physical,Faces,Edges,
            center,FixedQuaternion.Identity,coreAxis,(Fixed64)6,Fixed64.One,out _));
        Assert.True(WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(rigid,physical,Faces,Edges,
            center,FixedQuaternion.Identity,coreAxis,(Fixed64)6,Fixed64.One));
    }

    [Fact]
    public void ScaledHullRepresentative_ShouldRetainBothSignedExitsAndRejectUnrepresentableTranslatedWitnesses()
    {
        Vector3d[] authored=Array.ConvertAll(Cube,p=>p*4);
        var first=ScaledFrame(Vector3d.Zero);var second=ScaledFrame(Vector3d.Right*((Fixed64)3/2));
        Assert.True(WideOrientedBox.TryGetConvexHullContact(first,Vector3d.Zero,authored,Faces,Edges,
            second,Vector3d.Zero,authored,Faces,Edges,out var contact));
        Assert.Equal(Vector3d.Right,contact.Normal);Assert.Equal(Fixed64.Half,contact.Depth);
        Assert.Equal(Fixed64.One,contact.FirstAnchor.LocalPoint.X);
        Assert.Equal(Fixed64.Half,contact.SecondAnchor.LocalPoint.X);
        Vector3d outside=new(Fixed64.MinValue,Fixed64.Zero,Fixed64.Zero);
        Assert.False(WideOrientedBox.TryGetConvexHullContact(first,outside,authored,Faces,Edges,
            second,Vector3d.Zero,authored,Faces,Edges,out _));
        Assert.False(WideOrientedBox.TryGetConvexHullContact(first,Vector3d.Zero,authored,Faces,Edges,
            second,outside,authored,Faces,Edges,out _));
    }

    [Fact]
    public void ScaledTriangleHulls_ShouldRetainAnEdgeCrossSeparatingAxis()
    {
        Vector3d[] first={new(4,-8,8),new(0,12,4),new(-8,8,4)};
        Vector3d[] second={new(8,8,-8),new(-8,0,-12),new(-8,-4,8)};
        int[] faces={0,1,2},edges={0,1,1,2,2,0};var frame=ScaledFrame(Vector3d.Zero);
        Assert.False(WideOrientedBox.TryGetConvexHullPenetration(frame,first,faces,edges,
            frame,second,faces,edges,out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ScaledHullRoundedCorner_ShouldRetainTheFiniteEdgeOrVertexSupport(bool vertex)
    {
        Vector3d[] authored=Array.ConvertAll(Cube,p=>p*4),physical=Array.ConvertAll(Cube,p=>p*3);
        var frame=new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            Vector3d.One*((Fixed64)3/4),FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        Vector3d center=new((Fixed64)7/2,(Fixed64)7/2,vertex?(Fixed64)7/2:Fixed64.Zero);
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(frame,Vector3d.Zero,authored,Faces,Edges,
            center,FixedQuaternion.Identity,Vector3d.Up,Fixed64.Zero,Fixed64.One,out var contact));
        var cycledFaces=new int[Faces.Length];
        for(int index=0;index<Faces.Length;index+=3)
        {cycledFaces[index]=Faces[index+1];cycledFaces[index+1]=Faces[index+2];cycledFaces[index+2]=Faces[index];}
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(frame,Vector3d.Zero,authored,cycledFaces,Edges,
            center,FixedQuaternion.Identity,Vector3d.Up,Fixed64.Zero,Fixed64.One,out var cycled));
        Assert.Equal(contact.Normal,cycled.Normal);Assert.Equal(contact.Depth,cycled.Depth);
        Assert.True(WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(Vector3d.Zero,FixedQuaternion.Identity,
            physical,Faces,Edges,center,FixedQuaternion.Identity,Vector3d.Up,Fixed64.Zero,Fixed64.One,out var reference));
        Assert.Equal(reference.Normal,contact.Normal);Assert.Equal(reference.Depth,contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d hull));
        Assert.Equal(new Vector3d((Fixed64)3,(Fixed64)3,vertex?(Fixed64)3:Fixed64.Zero),hull);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d round));
        Assert.True(Vector3d.Dot(round-hull,contact.Normal)<=Fixed64.Zero);
    }

    private sealed class RetainedSupportDomain:IScaledConvexHullContactDomain
    {
        private static readonly RationalSegmentInterval[] Full={default};
        private static readonly RationalSegmentInterval[] Middle={new(Signed320.ExtendValue(Signed192.Signed(1)),
            Signed320.ExtendValue(Signed192.Signed(4)),Signed320.ExtendValue(Signed192.Signed(3)),Signed320.ExtendValue(Signed192.Signed(4)))};
        private readonly bool empty,retainMiddle,acceptFirst;
        internal RetainedSupportDomain(bool empty,bool retainMiddle,bool acceptFirst=false)
        {this.empty=empty;this.retainMiddle=retainMiddle;this.acceptFirst=acceptFirst;}
        public ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first,int start,int end,in WideDirection3d normal)=>
            empty?ReadOnlySpan<RationalSegmentInterval>.Empty:retainMiddle?Middle:Full;
        public bool ContainsPoint(bool first,int start,int end,in Signed832 n,in Signed832 d,in WideDirection3d normal)=>
            acceptFirst&&first||retainMiddle&&start!=end&&RationalSegmentInterval.CompareParameters(n,d,Middle[0].WideLowerNumerator,Middle[0].WideLowerDenominator)>=0
                &&RationalSegmentInterval.CompareParameters(n,d,Middle[0].WideUpperNumerator,Middle[0].WideUpperDenominator)<=0;
        public bool ContainsPoint(bool first,int start,int end,scoped ContactQuadratic n,scoped ContactQuadratic d,in WideDirection3d normal)=>
            acceptFirst&&first||retainMiddle&&start!=end&&Middle[0].Contains(n,d,ReadOnlySpan<ulong>.Empty);
    }

    private static ScaledRigidFrame3d ScaledFrame(Vector3d translation)=>new(translation,FixedQuaternion.Identity,
        Vector3d.Zero,Vector3d.One/4,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);

    private static ScaledRigidFrame3d Frame(Vector3d origin, FixedQuaternion rotation) =>
        new(origin, rotation, Vector3d.Zero, Vector3d.One, rotation, Vector3d.Zero, Vector3d.One);
}
