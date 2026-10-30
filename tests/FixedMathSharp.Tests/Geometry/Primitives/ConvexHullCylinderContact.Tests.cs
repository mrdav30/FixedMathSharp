using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class ConvexHullCylinderContactTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WholeHullRootContact_PreservesDirtyCallerHeadroomOnOneMiBStack(bool affine)
    {
        Vector3d extents=new((Fixed64)10,Fixed64.Half,Fixed64.Half);
        Vector3d[] points=Cube(affine?extents*Fixed64.Half:extents);
        var frame=new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            affine?Vector3d.One*Fixed64.Two:Vector3d.One,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        const long q=1920767767;
        FixedQuaternion rotation=new(Fixed64.Zero,Fixed64.Zero,Fixed64.FromRaw(-q),Fixed64.FromRaw(2*q));
        Vector3d center=new((Fixed64)(-5),(Fixed64)15.5m-Fixed64.FromFraction(15,1024),(Fixed64)20.5m-Fixed64.FromFraction(16,1024));
        var contacts=new FixedContactAnchors[2];var hits=new bool[2];Exception? failure=null;
        var worker=new Thread(()=>
        {
            try
            {
                for(int i=0;i<2;i++)hits[i]=HullContactWithDirtyCaller(frame,points,center,rotation,
                    i==0?0xA55A0FF012345678UL:0x5AA5F00FFEDCBA98UL,out contacts[i]);
            }
            catch(Exception exception){failure=exception;}
        },1024*1024);
        worker.Start();worker.Join();
        Assert.Null(failure);Assert.True(hits[0]);Assert.True(hits[1]);
        Assert.Equal(Fixed64.FromRaw(91988268),contacts[0].Depth);
        Assert.Equal(contacts[0],contacts[1]);
    }

    [Fact]
    public void ContainedCenterAdmission_IncludesBoundaryAndDeclinesExteriorOverlap()
    {
        var frame=ScaledRigidFrame3d.Rigid(Vector3d.Zero,FixedQuaternion.Identity);
        Vector3d[] points=Cube(Vector3d.One);
        foreach(Fixed64 x in new[]{Fixed64.Zero,Fixed64.One,(Fixed64)1.125m})
        {
            bool found=ConvexHullCylinderContact.TryGetContactWithContainedCenter(frame,points,WideOrientedBox.BoxHullTriangles,
                Vector3d.Right*x,FixedQuaternion.Identity,Fixed64.Half,Fixed64.Quarter,out var contact);
            Assert.Equal(x<=Fixed64.One,found);
            if(found)Assert.True(contact.Depth>Fixed64.Zero);
            else Assert.True(Contact(frame,points,Vector3d.Right*x,FixedQuaternion.Identity,Fixed64.Half,Fixed64.Quarter,out _));
        }
    }

    [Fact]
    public void DegenerateRadiusAndReversedBoundaryFaces_PreserveFiniteContacts()
    {
        var frame=ScaledRigidFrame3d.Rigid(Vector3d.Zero,FixedQuaternion.Identity);
        Vector3d[] points=Cube(Vector3d.One);
        FixedQuaternion rotation=new FixedQuaternion((Fixed64)1,(Fixed64)2,(Fixed64)3,(Fixed64)4).Normalized;
        Assert.True(Contact(frame,points,Vector3d.Zero,rotation,Fixed64.One,Fixed64.Zero,out var line));
        Assert.True(line.Depth>=Fixed64.One);
        int[] reversed=WideOrientedBox.BoxHullTriangles.ToArray();Array.Reverse(reversed);
        Assert.True(ConvexHullCylinderContact.TryGetContact(frame,points,reversed,Vector3d.Right,FixedQuaternion.Identity,
            Fixed64.Half,Fixed64.Quarter,out var boundary));
        Assert.Equal(Fixed64.Quarter,boundary.Depth);
        int[] rotated=new int[reversed.Length];
        for(int i=0;i<rotated.Length;i++)rotated[i]=WideOrientedBox.BoxHullTriangles[(i+12)%rotated.Length];
        Assert.True(ConvexHullCylinderContact.TryGetContact(frame,points,rotated,new Vector3d((Fixed64)1.25m,(Fixed64)1.25m,Fixed64.Zero),
            FixedQuaternion.Identity,Fixed64.Half,Fixed64.Quarter,out var corner));
        Assert.Equal(Vector3d.Right,corner.Normal);Assert.Equal(Fixed64.Zero,corner.Depth);
        Assert.True(corner.FirstAnchor.TryGetPoint(out Vector3d a));Assert.Equal(new Vector3d(1,1,0),a);
    }

    [Fact]
    public void BallCertificate_DeclinesPlanesThatDoNotBoundTheHull()
    {
        var frame=ScaledRigidFrame3d.Rigid(Vector3d.Zero,FixedQuaternion.Identity);
        Vector3d[] points=Cube(Vector3d.One);
        var hull=new CylinderHullSupport(frame,points,Vector3d.Zero,FixedQuaternion.Identity,Fixed64.Half,Fixed64.Quarter);
        var triangle=new FixedTriangle(points[0],points[2],points[3]);
        var geometry=new TriangleCircularGeometry(triangle,frame,hull,0,2,3);
        Span<ulong> values=stackalloc ulong[ConvexContactCandidate.Slots*CylinderContactAlgebra.Words],direction=stackalloc ulong[3*CylinderContactAlgebra.Words];
        Span<int> signs=stackalloc int[ConvexContactCandidate.Slots],directionSigns=stackalloc int[3];
        Assert.True(TriangleCylinderAnalyticFeatures.TryGetBest(geometry,default,default,values,signs,direction,directionSigns,
            out int gap,out _,out _,out _,hull));
        var candidate=new ConvexContactCandidate(values,signs,gap);
        Assert.False(hull.HasBallMinimumCertificate(frame,new[]{0,7,6},candidate));
        Assert.False(hull.HasBallMinimumCertificate(frame,new[]{0,0,0},candidate));
    }

    [Fact]
    public void AffineAnalyticRimEdge_RetainsExactMatchingWeightsAcrossVertexOrders()
    {
        var frame=new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One*Fixed64.Two,
            FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        var triangle=new FixedTriangle(new Vector3d(2,3,0),new Vector3d(3,2,0),new Vector3d(3,3,1));
        Span<ulong> normal=stackalloc ulong[3*CylinderContactAlgebra.Words];Span<int> signs=stackalloc int[3];
        CylinderContactAlgebra.WriteDirection(new WideDirection3d(WideArithmetic.Negate(Signed576.One),WideArithmetic.Negate(Signed576.One),default),normal,signs);
        for(int order=0;order<3;order++)
        {
            var geometry=new TriangleCircularGeometry(triangle,frame,Vector3d.Zero,FixedQuaternion.Identity,Signed192.Raw((Fixed64)10),(Fixed64)5);
            int mask=order==0?3:order==1?6:5;
            Assert.True(TriangleCylinderRimWitnesses.TryGetAnalyticPoint(geometry,triangle,mask,normal,signs,-1,geometry.Radius,out Vector3d point));
            Assert.Equal(new Vector3d(5,5,0),point);
            triangle=new FixedTriangle(triangle.C,triangle.A,triangle.B);
        }
    }

    [Fact]
    public void VariedContainedCylinders_MatchIndependentBoxMinimumAcrossTheWholeSupportFan()
    {
        Vector3d[] points=Cube(Vector3d.One);
        for(int seed=1;seed<=32;seed++)
        {
            FixedQuaternion rotation=new FixedQuaternion((Fixed64)(seed%5+1),(Fixed64)(seed%7-3),(Fixed64)3,(Fixed64)(seed%3+2)).Normalized;
            Vector3d center=new(Fixed64.FromFraction(seed%7-3,4),Fixed64.FromFraction(seed%5-2,3),Fixed64.FromFraction(seed%3-1,2));
            Fixed64 height=Fixed64.FromFraction(seed%8+1,4),radius=Fixed64.FromFraction(seed%6+1,3);
            var frame=ScaledRigidFrame3d.Rigid(Vector3d.Zero,FixedQuaternion.Identity);
            var box=new FixedOrientedBox(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.One);
            Assert.True(Contact(frame,points,center,rotation,height,radius,out var contact));
            Assert.True(box.TryGetCenteredCylinderContact(center,rotation,Vector3d.Up,height,radius,out var expected));
            Assert.Equal(expected.Depth,contact.Depth);
            AssertContact(frame,points,center,rotation,height,radius,contact);
        }
    }

    [Fact]
    public void IndependentAffinePlacement_PreservesObliqueFiniteWitnessesWithoutRoundingTheSourceFrame()
    {
        Vector3d reference=new(1000000,-1000000,1000000);
        Vector3d[] points=Cube(Vector3d.One*Fixed64.Two);
        for(int i=0;i<points.Length;i++)points[i]+=reference;
        FixedQuaternion placement=new FixedQuaternion((Fixed64)1,(Fixed64)2,(Fixed64)3,(Fixed64)4).Normalized;
        FixedQuaternion orientation=new FixedQuaternion((Fixed64)2,(Fixed64)3,(Fixed64)1,(Fixed64)5).Normalized;
        var frame=new ScaledRigidFrame3d(new Vector3d(2,-1,3),placement,new Vector3d((Fixed64).3m,(Fixed64).2m,(Fixed64).1m),
            new Vector3d(Fixed64.One+Fixed64.MinIncrement,(Fixed64).75m+Fixed64.MinIncrement,(Fixed64)1.25m+Fixed64.MinIncrement),
            orientation,reference,new Vector3d(Fixed64.One+Fixed64.MinIncrement,Fixed64.One,Fixed64.One));
        Assert.True(frame.TryGetPoint(reference,out Vector3d center));
        FixedQuaternion rotation=new FixedQuaternion((Fixed64)3,(Fixed64)1,(Fixed64)4,(Fixed64)2).Normalized;
        Assert.True(Contact(frame,points,center,rotation,Fixed64.One,Fixed64.Half,out var contact));
        AssertContact(frame,points,center,rotation,Fixed64.One,Fixed64.Half,contact);
    }

    [Theory]
    [InlineData(-1,0)]
    [InlineData(1,0)]
    [InlineData(0,-1)]
    [InlineData(0,1)]
    public void BallCertificate_PreservesSubRawCapSideRanking(int radiusRaw, int heightRaw)
    {
        Fixed64 radius=Fixed64.Quarter+Fixed64.FromRaw(radiusRaw),height=Fixed64.Half+Fixed64.FromRaw(heightRaw);
        var frame=ScaledRigidFrame3d.Rigid(Vector3d.Zero,FixedQuaternion.Identity);
        Assert.True(Contact(frame,Cube(Vector3d.One),Vector3d.Zero,FixedQuaternion.Identity,height,radius,out var contact));
        bool side=radius.m_rawValue*2<height.m_rawValue;
        Assert.Equal(side,contact.Normal.Y==Fixed64.Zero);
        Assert.Equal((Fixed64)1.25m+Fixed64.FromRaw(radiusRaw<0?radiusRaw:0),contact.Depth);
    }

    [Fact]
    public void FiniteWitnessAdmission_RejectsMissingSideIntersectionsAndPreservesCapEndpoints()
    {
        Span<ulong> normal = stackalloc ulong[3 * CylinderContactAlgebra.Words]; Span<int> signs = stackalloc int[3];
        CylinderContactAlgebra.WriteDirection(new WideDirection3d(Signed576.One, default, default), normal, signs);
        foreach (FixedTriangle triangle in new[]
        {
            new FixedTriangle(new Vector3d(1,0,2),new Vector3d(1,1,3),new Vector3d(1,-1,3)),
            new FixedTriangle(new Vector3d(1,2,0),new Vector3d(1,3,1),new Vector3d(1,4,1)),
            new FixedTriangle(new Vector3d(1,2,-1),new Vector3d(1,2,1),new Vector3d(1,3,1))
        })
        {
            var geometry = new TriangleCircularGeometry(triangle, Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, Signed192.Raw(Fixed64.Two), Fixed64.One);
            Assert.False(TriangleCylinderWitnesses.TryGetSidePoint(geometry, triangle, 7, normal, signs, out _, out _, out _));
        }
        foreach (int cap in new[] {-1,1})
        {
            var triangle = new FixedTriangle(new Vector3d(1,cap,0),new Vector3d(1,cap,1),new Vector3d(1,cap+1,1));
            var geometry = new TriangleCircularGeometry(triangle, Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, Signed192.Raw(Fixed64.Two), Fixed64.One);
            Assert.True(TriangleCylinderWitnesses.TryGetSidePoint(geometry, triangle, 7, normal, signs, out Vector3d point, out Fixed64 axial, out int endpoint));
            Assert.Equal(triangle.A, point); Assert.Equal((Fixed64)cap, axial); Assert.Equal(-cap, endpoint);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void AnalyticRimWitness_AdmitsOnlyMatchingFiniteVertexAndFaceFeatures(int order)
    {
        Span<ulong> normal = stackalloc ulong[3 * CylinderContactAlgebra.Words]; Span<int> signs = stackalloc int[3];
        foreach (Vector3d direction in new[] { new Vector3d(-3,-14,-4), new Vector3d(0,-14,-5), new Vector3d(0,0,-5) })
        {
            Vector3d point = direction.X == Fixed64.Zero ? new Vector3d(0,5,5) : new Vector3d(3,5,4);
            CylinderContactAlgebra.WriteDirection(new WideDirection3d(Signed576.ExtendValue(Signed192.Raw(direction.X)),
                Signed576.ExtendValue(Signed192.Raw(direction.Y)), Signed576.ExtendValue(Signed192.Raw(direction.Z))), normal, signs);
            foreach (bool matched in new[] {true,false})
            {
                Vector3d a = point + (matched ? Vector3d.Zero : Vector3d.Up * Fixed64.Quarter);
                var triangle = new FixedTriangle(a,a+new Vector3d(1,4,1),a+new Vector3d(2,1,1));
                for (int i=0;i<order;i++) triangle = new FixedTriangle(triangle.C,triangle.A,triangle.B);
                var geometry = new TriangleCircularGeometry(triangle,Vector3d.Zero,FixedQuaternion.Identity,
                    Vector3d.Zero,FixedQuaternion.Identity,Signed192.Raw((Fixed64)10),(Fixed64)5);
                Assert.Equal(matched,TriangleCylinderRimWitnesses.TryGetAnalyticPoint(geometry,triangle,1<<order,
                    normal,signs,-1,geometry.Radius,out Vector3d witness));
                if(matched) Assert.Equal(point,witness);
            }
        }
        CylinderContactAlgebra.WriteDirection(new WideDirection3d(WideArithmetic.Negate(Signed576.One),WideArithmetic.Negate(Signed576.One),default),normal,signs);
        foreach (bool matched in new[] {true,false})
        {
            int z = matched ? -1 : 4;
            var triangle = new FixedTriangle(new Vector3d(4,6,z),new Vector3d(6,4,z),new Vector3d(5,5,z+2));
            for(int i=0;i<order;i++) triangle = new FixedTriangle(triangle.C,triangle.A,triangle.B);
            var geometry = new TriangleCircularGeometry(triangle,Vector3d.Zero,FixedQuaternion.Identity,
                Vector3d.Zero,FixedQuaternion.Identity,Signed192.Raw((Fixed64)10),(Fixed64)5);
            Assert.Equal(matched,TriangleCylinderRimWitnesses.TryGetAnalyticPoint(geometry,triangle,7,normal,signs,-1,geometry.Radius,out _));
            Assert.False(TriangleCylinderRimWitnesses.TryGetAnalyticPoint(geometry,triangle,3,normal,signs,-1,geometry.Radius,out _));
        }
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void Tetrahedron_RimRootPreservesTouchAndAdjacentSeparation(int offsetRaw)
    {
        Fixed64 e = Fixed64.MinIncrement;
        Vector3d[] points =
        {
            new((Fixed64)1-e, (Fixed64)6+e, (Fixed64)2-e),
            new((Fixed64)5+e, (Fixed64)4, (Fixed64)6+e),
            new((Fixed64)6, (Fixed64)19+e, (Fixed64)8), new(11,23,9)
        };
        int[] triangles = { 0,1,2, 0,3,1, 0,2,3, 1,3,2 };
        Vector3d translation = new Vector3d(3,14,4) * Fixed64.FromRaw(offsetRaw);
        var frame = ScaledRigidFrame3d.Rigid(translation, FixedQuaternion.Identity);
        Assert.Equal(offsetRaw <= 0, ConvexHullCylinderContact.TryGetContact(frame, points, triangles, Vector3d.Zero,
            FixedQuaternion.Identity, (Fixed64)10+e, (Fixed64)5, out var contact));
        if (offsetRaw == 0)
        {
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d a));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d b));
            Assert.Equal(new Vector3d(3,5,4), a); Assert.Equal(a,b);
        }
    }

    [Fact]
    public void WholeHull_RejectsSmoothCornerSeparationAndIncompleteFiniteWitnesses()
    {
        Vector3d[] points = { new((Fixed64)5.25m,(Fixed64)5.25m,Fixed64.Zero),
            new((Fixed64)4.75m,(Fixed64)6,Fixed64.Zero), new((Fixed64)6,(Fixed64)4.75m,Fixed64.Zero), new(6,6,1) };
        int[] tetrahedron = { 0,1,2, 0,3,1, 0,2,3, 1,3,2 };
        var frame = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        Assert.False(ConvexHullCylinderContact.TryGetContact(frame, points, tetrahedron, Vector3d.Zero,
            FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out _));
        Assert.True(Contact(frame, Cube(Vector3d.One), Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Half, Fixed64.Zero, out var axisContact));
        Assert.Equal(Fixed64.One, axisContact.Depth);
        // An incomplete topology cannot supply a witness on the omitted side
        // of the hull. Decline it rather than project outside its finite face.
        Assert.False(ConvexHullCylinderContact.TryGetContact(frame, Cube(Vector3d.One), new[] {2,6,7},
            new Vector3d(Fixed64.Half,Fixed64.Zero,-Fixed64.Half), FixedQuaternion.Identity, Fixed64.Half, Fixed64.Quarter, out _));
    }

    [Theory]
    [InlineData(false, 1)]
    [InlineData(false, -1)]
    [InlineData(true, 1)]
    [InlineData(true, -1)]
    public void EdgeRim_GlobalRootMatchesIndependentBoxContact(bool affine, int sign)
    {
        Vector3d extents = new((Fixed64)10, Fixed64.Half, Fixed64.Half);
        Vector3d[] points = Cube(affine ? extents * Fixed64.Half : extents);
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            affine ? Vector3d.One * Fixed64.Two : Vector3d.One, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        const long q = 1920767767;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-q), Fixed64.FromRaw(2 * q));
        Vector3d center = new Vector3d((Fixed64)(-5), (Fixed64)15.5m - Fixed64.FromFraction(15, 1024),
            (Fixed64)20.5m - Fixed64.FromFraction(16, 1024)) * (Fixed64)sign;
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, extents);
        Assert.True(box.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up, (Fixed64)10, (Fixed64)25, out var expected));
        Assert.True(Contact(frame, points, center, rotation, (Fixed64)10, (Fixed64)25, out var contact));
        Assert.Equal(Fixed64.FromRaw(91988268), contact.Depth);
        Assert.Equal(expected.Depth, contact.Depth);
        Assert.Equal(expected.Normal, contact.Normal);
        AssertContact(frame, points, center, rotation, (Fixed64)10, (Fixed64)25, contact);
    }

    [Fact]
    public void RootTouch_AndFullDomainClampedExitRetainTheirExactClassification()
    {
        const long q = 1753413056;
        FixedQuaternion rotation = new(Fixed64.FromRaw(q), Fixed64.Zero, Fixed64.FromRaw(-q), Fixed64.FromRaw(2 * q));
        var frame = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        Assert.True(Contact(frame, Cube(new Vector3d(10, 1, 1)), new Vector3d(0, 4, 4), rotation, (Fixed64)6, (Fixed64)3, out var touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
        Assert.True(Contact(frame, Cube(Vector3d.One * Fixed64.MaxValue), Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue, out var clamped));
        Assert.Equal(Fixed64.MaxValue, clamped.Depth);
        Assert.True(clamped.DepthIsClamped);
        Assert.False(ConvexHullCylinderContact.TryGetContact(frame, ReadOnlySpan<Vector3d>.Empty,
            ReadOnlySpan<int>.Empty, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, Fixed64.One, out _));
    }

    [Fact]
    public void AffineCapPole_RetainsCompleteBarycentricSquaresAcrossIndependentYawFrames()
    {
        FixedQuaternion placement = FixedQuaternion.FromAxisAngle(Vector3d.Up, (Fixed64).31m);
        FixedQuaternion geometry = FixedQuaternion.FromAxisAngle(Vector3d.Up, (Fixed64).47m);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, (Fixed64).73m);
        Vector3d reference = new(1000000, -1000000, 1000000);
        Vector3d[] points = Cube(new Vector3d(Fixed64.Two + Fixed64.FromRaw(1), Fixed64.Quarter + Fixed64.FromRaw(3), Fixed64.Two + Fixed64.FromRaw(5)));
        for (int i = 0; i < points.Length; i++) points[i] += reference;
        var frame = new ScaledRigidFrame3d(new Vector3d(2, -1, 3), placement,
            new Vector3d(Fixed64.FromRaw(12345), Fixed64.FromRaw(54321), Fixed64.FromRaw(67891)),
            new Vector3d(Fixed64.One + Fixed64.FromRaw(1), Fixed64.One + Fixed64.FromRaw(3), Fixed64.One + Fixed64.FromRaw(5)),
            geometry, reference, new Vector3d(Fixed64.One + Fixed64.FromRaw(7), Fixed64.One + Fixed64.FromRaw(9), Fixed64.One + Fixed64.FromRaw(11)));
        Assert.True(frame.TryGetPoint(reference, out Vector3d center));
        Assert.True(Contact(frame, points, center, rotation, Fixed64.Half, Fixed64.Quarter, out var contact));
        Assert.True(contact.Normal == Vector3d.Up || contact.Normal == Vector3d.Down);
        AssertContact(frame, points, center, rotation, Fixed64.Half, Fixed64.Quarter, contact);
        bool exceedsOldSquareBank = false;
        ReadOnlySpan<int> indices = WideOrientedBox.BoxHullTriangles;
        Span<ulong> weights = stackalloc ulong[3 * CylinderContactAlgebra.Words];
        foreach(Vector3d candidateCenter in new[]{center,center+Vector3d.Right*(Fixed64)3})
        {
            var hull = new CylinderHullSupport(frame, points, candidateCenter, rotation, Fixed64.Half, Fixed64.Quarter);
            for (int i = 0; i < indices.Length; i += 3)
            {
                var triangle = new FixedTriangle(points[indices[i]], points[indices[i + 1]], points[indices[i + 2]]);
                if (triangle.A.Y != triangle.B.Y || triangle.A.Y != triangle.C.Y) continue;
                var chart = new TriangleCircularGeometry(triangle, frame, hull, indices[i], indices[i + 1], indices[i + 2]);
                TriangleCylinderWitnesses.GetCapWeights(chart, 7, weights);
                BigInteger nx = 0, nz = 0, denominator = 0;
                for (int vertex = 0; vertex < 3; vertex++)
                {
                    BigInteger weight = Magnitude(weights.Slice(vertex * CylinderContactAlgebra.Words, CylinderContactAlgebra.Words));
                    nx += weight * Integer(chart.Vertex(vertex).X); nz += weight * Integer(chart.Vertex(vertex).Z); denominator += weight;
                }
                BigInteger limit = denominator * Integer(chart.Radius);
                exceedsOldSquareBank |= BigInteger.Max(nx * nx + nz * nz, limit * limit).GetBitLength() > 2560;
                Assert.Equal(nx*nx+nz*nz<=limit*limit,TriangleCylinderWitnesses.TryGetCapPoint(chart,triangle,7,out _,out _));
            }
        }
        Assert.True(exceedsOldSquareBank, "The fixture must exercise squares wider than the former 40-word admission bank.");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void InclusiveHullPointAdmission_PreservesFiniteBoundariesAndWinding(bool affine)
    {
        Vector3d[] points = Cube(Vector3d.One);
        Vector3d scale = affine ? Vector3d.One * Fixed64.Two : Vector3d.One;
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, scale,
            FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        int[] triangles = WideOrientedBox.BoxHullTriangles.ToArray();
        foreach (bool reverse in new[] { false, true })
        {
            if (reverse) Array.Reverse(triangles);
            foreach (Vector3d boundary in new[] { new Vector3d(1, 0, 0), new Vector3d(1, 1, 0), new Vector3d(1, 1, 1) })
            {
                Vector3d p = boundary * scale.X;
                Assert.True(WideOrientedBox.ContainsConvexHullPoint(frame, points, triangles, p));
                Assert.False(WideOrientedBox.ContainsConvexHullPoint(frame, points, triangles, p, strict: true));
            }
            foreach (int raw in new[] { -1, 0, 1 })
                Assert.Equal(raw <= 0, WideOrientedBox.ContainsConvexHullPoint(frame, points, triangles,
                    Vector3d.Right * (scale.X + Fixed64.FromRaw(raw))));
            Assert.False(WideOrientedBox.ContainsConvexHullPoint(frame, points, triangles, new Vector3d(scale.X, scale.Y + Fixed64.One, Fixed64.Zero)));
        }
    }

    [Theory]
    [InlineData(0, 0, 0)]
    [InlineData(0, 1, 0)]
    [InlineData(1, 0, 0)]
    [InlineData(0, 0, 1)]
    [InlineData(-1, 0, 0)]
    [InlineData(0, -1, 0)]
    [InlineData(0, 0, -1)]
    public void Cube_PreservesFiniteWitnessesAndExactClearance(int x, int y, int z)
    {
        Vector3d[] points = Cube(Vector3d.One);
        Vector3d center = new Vector3d(x, y, z) * Fixed64.Half;
        var frame = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        Assert.True(Contact(frame, points, center, FixedQuaternion.Identity, Fixed64.Half, Fixed64.Quarter, out var contact));
        Assert.Equal(x == 0 && y == 0 && z == 0 ? (Fixed64)1.25m : (Fixed64).75m, contact.Depth);
        AssertContact(frame, points, center, FixedQuaternion.Identity, Fixed64.Half, Fixed64.Quarter, contact);
        foreach (int raw in new[] { -1, 0, 1 })
        {
            Vector3d moved = center + contact.Normal * (contact.Depth + Fixed64.FromRaw(raw));
            Assert.Equal(raw < 0, WideOrientedBox.DoesCenteredCylinderPenetrateConvexHull(moved,
                FixedQuaternion.Identity, Fixed64.Half, Fixed64.Quarter, frame, points, WideOrientedBox.BoxHullTriangles, true));
            Assert.Equal(raw <= 0, Contact(frame, points, moved, FixedQuaternion.Identity, Fixed64.Half, Fixed64.Quarter, out _));
        }
    }

    [Fact]
    public void SkewHull_ContainmentMinimumIsAnEdgeSideDirectionRatherThanAFaceExit()
    {
        Vector3d[] points = Cube(new Vector3d(1, 1, 4));
        for (int i = 0; i < points.Length; i++)
            points[i] = new Vector3d(points[i].X, (Fixed64)1.25m * points[i].Y - (Fixed64).75m * points[i].X, points[i].Z);
        var frame = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -Fixed64.One,
            FixedMath.Sqrt((Fixed64)5) - Fixed64.Two).Normalized;
        Fixed64 height = (Fixed64)4, radius = (Fixed64).01m;
        Assert.True(Contact(frame, points, Vector3d.Zero, rotation, height, radius, out var contact));
        Assert.True(contact.Depth < (Fixed64)1.2m);
        AssertContact(frame, points, Vector3d.Zero, rotation, height, radius, contact);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueHull_ExactFeaturesClearWholeShapeAcrossRigidAndAffineCharts(bool affine)
    {
        Vector3d origin = new(2, -1, 3);
        for (int seed = 1; seed <= 16; seed++)
        {
            FixedQuaternion hullRotation = new FixedQuaternion((Fixed64)(seed + 1), (Fixed64)3, (Fixed64)(seed % 4 + 1), (Fixed64)7).Normalized;
            FixedQuaternion cylinderRotation = new FixedQuaternion((Fixed64)(seed % 3 + 1), (Fixed64)5, (Fixed64)2, (Fixed64)(seed + 4)).Normalized;
            Vector3d scale = affine ? new Vector3d((Fixed64)1.5m, (Fixed64).75m, (Fixed64)1.25m) : Vector3d.One;
            Vector3d reference = affine ? new Vector3d((Fixed64)1000000, (Fixed64)(-1000000), (Fixed64)1000000) : Vector3d.Zero;
            Vector3d[] points = Cube(Vector3d.One);
            for (int i = 0; i < points.Length; i++) points[i] += reference;
            var frame = new ScaledRigidFrame3d(origin, hullRotation, Vector3d.Zero, scale, hullRotation, reference, Vector3d.One);
            Assert.True(Contact(frame, points, origin, cylinderRotation, Fixed64.Half, Fixed64.Quarter, out var contact), $"seed {seed}, affine {affine}");
            var box = new FixedOrientedBox(origin, hullRotation, scale);
            Assert.True(box.TryGetCenteredCylinderContact(origin, cylinderRotation, Vector3d.Up,
                Fixed64.Half, Fixed64.Quarter, out var expected));
            Assert.Equal(expected.Depth, contact.Depth);
            AssertContact(frame, points, origin, cylinderRotation, Fixed64.Half, Fixed64.Quarter, contact);
        }
    }

    private static void AssertContact(in ScaledRigidFrame3d frame, Vector3d[] points, Vector3d center,
        FixedQuaternion rotation, Fixed64 height, Fixed64 radius, FixedContactAnchors contact)
    {
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d a));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d b));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Assert.True((offset - contact.Normal * contact.Depth).Magnitude <= Fixed64.FromRaw(64), $"offset {offset}, normal {contact.Normal}, depth {contact.Depth}");
        Assert.Equal(center, contact.SecondAnchor.Origin);
        Assert.Equal(rotation, contact.SecondAnchor.Rotation);
        Vector3d localCylinder = contact.SecondAnchor.LocalPoint + contact.SecondAnchor.LocalDisplacement;
        Assert.True(FixedMath.Abs(localCylinder.Y) <= height * Fixed64.Half + Fixed64.FromRaw(1));
        // Two once-rounded radial coordinates contribute at most 2R raw
        // units plus two raw units for their squared fixed-point products.
        Fixed64 radialRounding = radius * Fixed64.FromRaw(2) + Fixed64.FromRaw(2);
        Assert.True(localCylinder.X * localCylinder.X + localCylinder.Z * localCylinder.Z <= radius * radius + radialRounding);
        Assert.True(WideOrientedBox.DoesCenteredCylinderPenetrateConvexHull(center + contact.Normal * (contact.Depth - Fixed64.FromRaw(64)),
            rotation, height, radius, frame, points, WideOrientedBox.BoxHullTriangles, true));
        Assert.False(WideOrientedBox.DoesCenteredCylinderPenetrateConvexHull(center + contact.Normal * (contact.Depth + Fixed64.FromRaw(64)),
            rotation, height, radius, frame, points, WideOrientedBox.BoxHullTriangles, true));
        // A finite hull witness must lie on at least one actual face triangle.
        bool onFace = false;
        ReadOnlySpan<int> triangles = WideOrientedBox.BoxHullTriangles;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            frame.TryGetPoint(points[triangles[i]], out Vector3d p);
            frame.TryGetPoint(points[triangles[i + 1]], out Vector3d q);
            frame.TryGetPoint(points[triangles[i + 2]], out Vector3d r);
            Vector3d nearest = new FixedTriangle(p, q, r).ClosestPoint(a);
            if ((nearest - a).Magnitude <= Fixed64.FromRaw(64)) onFace = true;
        }
        Assert.True(onFace, $"hull witness {a}");
    }

    private static bool Contact(in ScaledRigidFrame3d frame, Vector3d[] points, Vector3d center,
        FixedQuaternion rotation, Fixed64 height, Fixed64 radius, out FixedContactAnchors contact) =>
        ConvexHullCylinderContact.TryGetContact(frame, points, WideOrientedBox.BoxHullTriangles, center, rotation, height, radius, out contact);

    private static Vector3d[] Cube(Vector3d extents)
    {
        var points = new Vector3d[8];
        WideOrientedBox.GetBoxHullVertices(extents, points);
        return points;
    }

    private static BigInteger Integer(Signed576 value)
    {
        Span<ulong> words = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(value, words);
        return value.Sign < 0 ? -Magnitude(words) : Magnitude(words);
    }

    private static BigInteger Magnitude(ReadOnlySpan<ulong> words)
    {
        BigInteger result = 0;
        for (int i = words.Length - 1; i >= 0; i--) result = (result << 64) + words[i];
        return result;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool HullContactWithDirtyCaller(in ScaledRigidFrame3d frame,Vector3d[] points,Vector3d center,
        FixedQuaternion rotation,ulong seed,out FixedContactAnchors contact)
    {
        Span<ulong> caller=stackalloc ulong[8192];
        for(int i=0;i<caller.Length;i++)caller[i]=seed^unchecked(0x9E3779B97F4A7C15UL*(ulong)(i+1));
        bool found=Contact(frame,points,center,rotation,(Fixed64)10,(Fixed64)25,out contact);
        for(int i=0;i<caller.Length;i++)
            if(caller[i]!=(seed^unchecked(0x9E3779B97F4A7C15UL*(ulong)(i+1))))throw new InvalidOperationException("Hull contact changed the live caller buffer.");
        return found;
    }
}
