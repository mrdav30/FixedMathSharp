//=======================================================================
// WideTriangleConeIntersection.ScaledFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact source-frame ingress for finite triangle/cone minimum-axial queries.</content>
internal static partial class WideTriangleConeIntersection
{
    private const int ScaledQueryWords = 32;

    internal static bool TryGetMinimumAxialPoint(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        Vector3d apex, Vector3d axis, Fixed64 height, Fixed64 radius,
        out FixedPointAnchor point)
    {
        if (frame.IsRigid)
        {
            bool found = TryGetMinimumAxialPoint(triangle,frame.RigidOrigin,frame.Rotation,apex,axis,height,radius,out Vector3d local);
            point = found ? new FixedPointAnchor(frame.RigidOrigin,frame.Rotation,local) : default;
            return found;
        }
        bool hasPoint=false;point=default;Fixed64 best=Fixed64.MaxValue;
        for(int edge=0;edge<3;edge++)
        {
            if(!TryGetScaledConeEdgePoint(triangle.GetEdge(edge),frame,apex,axis,height,radius,out FixedPointAnchor candidate))continue;
            Fixed64 axial=GetScaledConeAxial(candidate,apex,axis,height);
            if(hasPoint&&axial>=best)continue;
            hasPoint=true;point=candidate;best=axial;
        }
        if(TryGetScaledConeFacePoint(triangle,frame,apex,axis,height,radius,hasPoint,out FixedPointAnchor face,out Fixed64 faceAxial)
            &&(!hasPoint||faceAxial<best)){point=face;hasPoint=true;}
        return hasPoint;
    }

    private static Fixed64 GetScaledConeAxial(in FixedPointAnchor point,Vector3d apex,Vector3d axis,Fixed64 height) =>
        FixedMath.Min(point.ProjectNonNegativeOffsetFrom(new FixedPointAnchor(apex,FixedQuaternion.Identity,Vector3d.Zero),axis),height);

    private static WideDirection3d GetScaledConeRelativePoint(in RationalPoint3d point,Vector3d apex) => new(
        WideArithmetic.SubtractSigned576(Signed576.ExtendValue(point.X),WideArithmetic.MultiplySigned320(point.Denominator,Signed192.Raw(apex.X))),
        WideArithmetic.SubtractSigned576(Signed576.ExtendValue(point.Y),WideArithmetic.MultiplySigned320(point.Denominator,Signed192.Raw(apex.Y))),
        WideArithmetic.SubtractSigned576(Signed576.ExtendValue(point.Z),WideArithmetic.MultiplySigned320(point.Denominator,Signed192.Raw(apex.Z))));

    private static bool TryGetScaledConeEdgePoint(in FixedSegment edge,in ScaledRigidFrame3d frame,
        Vector3d apex,Vector3d axis,Fixed64 height,Fixed64 radius,out FixedPointAnchor point)
    {
        point=default;
        WideDirection3d first=GetScaledConeRelativePoint(frame.GetPoint(edge.Start),apex);
        WideDirection3d last=GetScaledConeRelativePoint(frame.GetPoint(edge.End),apex);
        WideDirection3d delta=new(WideArithmetic.SubtractSigned576(last.X,first.X),
            WideArithmetic.SubtractSigned576(last.Y,first.Y),WideArithmetic.SubtractSigned576(last.Z,first.Z));
        var direction=new WideDirection3d(new WideAxis3(Signed320.ExtendValue(Signed192.Raw(axis.X)),
            Signed320.ExtendValue(Signed192.Raw(axis.Y)),Signed320.ExtendValue(Signed192.Raw(axis.Z))));
        Signed832 q=direction.SquaredLength, startAxial=WideDirection3d.Dot(first,direction), velocity=WideDirection3d.Dot(delta,direction);
        Signed832 start=WideArithmetic.MultiplySigned832(startAxial,Scale), end=WideArithmetic.MultiplySigned832(WideArithmetic.AddSigned832(startAxial,velocity),Scale);
        Signed832 maximum=default;
        // The denominator may exceed192 after independent placement rotations.
        // Keep the complete product in the borrowed coefficient bank below.
        const int words=ScaledQueryWords;
        Span<ulong> values=stackalloc ulong[14*words];Span<int> signs=stackalloc int[14];
        var a=ContactQuadratic.At(values,signs,0,words);var b=ContactQuadratic.At(values,signs,1,words);
        var c=ContactQuadratic.At(values,signs,2,words);var work=ContactQuadratic.At(values,signs,3,words);
        var term=ContactQuadratic.At(values,signs,4,words);var axial=ContactQuadratic.At(values,signs,5,words);
        var radial=ContactQuadratic.At(values,signs,6,words);
        work.Set(q);ContactQuadratic.Scale(work,Signed576.ExtendValue(frame.Denominator),term);
        ContactQuadratic.Scale(term,Signed576.ExtendValue(Signed192.Raw(height)),work);
        maximum=PackScaledCone832(work);
        Signed832 lowerN=default,lowerD=Signed832.ExtendValue(Signed192.One),upperN=lowerD,upperD=lowerD;
        if(!WideTriangleRelations.ClipScaledHalfPlane(start,end,ref lowerN,ref lowerD,ref upperN,ref upperD)
            ||!WideTriangleRelations.ClipScaledHalfPlane(WideArithmetic.SubtractSigned832(maximum,start),
                WideArithmetic.SubtractSigned832(maximum,end),ref lowerN,ref lowerD,ref upperN,ref upperD))return false;
        BuildScaledConeEdgeCoefficient(delta,delta,velocity,velocity,q,height,radius,a,work,term,axial,radial);
        BuildScaledConeEdgeCoefficient(first,delta,startAxial,velocity,q,height,radius,b,work,term,axial,radial);
        BuildScaledConeEdgeCoefficient(first,first,startAxial,startAxial,q,height,radius,c,work,term,axial,radial);
        if(!WideFiniteConeIntersection.TrySolveBoundedUnitPolynomial(a,b,c,lowerN,lowerD,upperN,upperD,
            Fixed64.MaxValue,out Fixed64 entry,out Fixed64 exit))return false;
        Fixed64 parameter=velocity.Sign<0?exit:entry;
        work.Set(Signed576.ExtendValue(Signed192.Raw(parameter)));term.Set(Signed576.ExtendValue(MaximumParameter));
        return WideTriangleRelations.TryGetSegmentPointAnchor(frame,edge,work,term,apex,out point);
    }

    private static void BuildScaledConeEdgeCoefficient(in WideDirection3d first,in WideDirection3d second,
        Signed832 firstAxial,Signed832 secondAxial,Signed832 q,Fixed64 height,Fixed64 radius,
        scoped ContactQuadratic output,scoped ContactQuadratic work,scoped ContactQuadratic term,
        scoped ContactQuadratic axial,scoped ContactQuadratic radial)
    {
        work.Set(firstAxial);ContactQuadratic.Scale(work,secondAxial,axial);
        work.Set(WideDirection3d.Dot(first,second));ContactQuadratic.Scale(work,q,radial);radial.Add(axial,-1);
        ContactQuadratic.Scale(radial,q,work);ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(height)),term);
        ContactQuadratic.Scale(term,Signed576.ExtendValue(Signed192.Raw(height)),output);
        ContactQuadratic.Scale(axial,Signed576.ExtendValue(Scale),work);ContactQuadratic.Scale(work,Signed576.ExtendValue(Scale),term);
        ContactQuadratic.Scale(term,Signed576.ExtendValue(Signed192.Raw(radius)),work);
        ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(radius)),term);output.Add(term,-1);
    }

    private static bool TryGetScaledConeFacePoint(in FixedTriangle triangle,in ScaledRigidFrame3d frame,
        Vector3d apex,Vector3d axis,Fixed64 height,Fixed64 radius,bool hasEdge,
        out FixedPointAnchor point,out Fixed64 axial)
    {
        point=default;axial=default;
        WideDirection3d normal=WideTriangleRelations.GetScaledTriangleLocalNormal(triangle,frame).Transform(frame.GeometryBasis);
        if(normal.IsZero)return false;
        var direction=new WideDirection3d(new WideAxis3(Signed320.ExtendValue(Signed192.Raw(axis.X)),
            Signed320.ExtendValue(Signed192.Raw(axis.Y)),Signed320.ExtendValue(Signed192.Raw(axis.Z))));
        Signed832 normalSquare=normal.SquaredLength,alignment=WideDirection3d.Dot(normal,direction);
        Signed832 plane=WideDirection3d.Dot(normal,GetScaledConeRelativePoint(frame.GetPoint(triangle.A),apex));
        Signed832 q=direction.SquaredLength;
        const int words=ScaledQueryWords;
        Span<ulong> values=stackalloc ulong[18*words];Span<int> signs=stackalloc int[18];
        var a=ContactQuadratic.At(values,signs,0,words);var b=ContactQuadratic.At(values,signs,1,words);
        var c=ContactQuadratic.At(values,signs,2,words);var p=ContactQuadratic.At(values,signs,3,words);
        var work=ContactQuadratic.At(values,signs,4,words);var term=ContactQuadratic.At(values,signs,5,words);
        var axisN=ContactQuadratic.At(values,signs,6,words);var axisD=ContactQuadratic.At(values,signs,7,words);
        var ratio=ContactQuadratic.At(values,signs,8,words);
        work.Set(normalSquare);ContactQuadratic.Scale(work,q,p);
        work.Set(alignment);ContactQuadratic.Scale(work,alignment,term);p.Add(term,-1);
        axisN.Set(plane);ContactQuadratic.Scale(axisN,Signed576.ExtendValue(Scale),work);work.CopyTo(axisN);
        axisD.Set(alignment);ContactQuadratic.Scale(axisD,Signed576.ExtendValue(frame.Denominator),work);
        ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(height)),axisD);
        if(axisD.Signs[0]<0){axisN.MultiplySign(-1);axisD.MultiplySign(-1);}
        bool crossesAxis=axisD.Signs[0]!=0&&axisN.Signs[0]>=0
            &&WideArithmetic.CompareMagnitudeEqualLength(axisN.Rational,axisD.Rational)<=0;
        if(p.Signs[0]==0)
        {
            if(hasEdge||!crossesAxis)return false;
            return TryGetScaledConeAxisPoint(triangle,frame,apex,axis,alignment,plane,true,out point,out axial);
        }
        work.Set(alignment);ContactQuadratic.Scale(work,alignment,term);ContactQuadratic.Scale(term,q,a);
        ContactQuadratic.Scale(a,Signed576.ExtendValue(Signed192.Raw(height)),work);
        ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(height)),a);
        ContactQuadratic.Scale(p,Signed576.ExtendValue(Signed192.Raw(radius)),work);
        ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(radius)),term);
        ContactQuadratic.Scale(term,Signed576.ExtendValue(Scale),work);ContactQuadratic.Scale(work,Signed576.ExtendValue(Scale),term);a.Add(term,-1);
        ContactQuadratic.Scale(a,Signed576.ExtendValue(frame.Denominator),work);ContactQuadratic.Scale(work,Signed576.ExtendValue(frame.Denominator),a);
        work.Set(alignment);ContactQuadratic.Scale(work,plane,term);ContactQuadratic.Scale(term,q,b);
        ContactQuadratic.Scale(b,Signed576.ExtendValue(Scale),work);ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(height)),term);
        ContactQuadratic.Scale(term,Signed576.ExtendValue(frame.Denominator),b);b.MultiplySign(-1);
        work.Set(plane);ContactQuadratic.Scale(work,plane,term);ContactQuadratic.Scale(term,q,c);
        ContactQuadratic.Scale(c,Signed576.ExtendValue(Scale),work);ContactQuadratic.Scale(work,Signed576.ExtendValue(Scale),c);
        Signed832 one=Signed832.ExtendValue(Signed192.One);
        if(!WideFiniteConeIntersection.TrySolveBoundedUnitPolynomial(a,b,c,default,one,one,one,Fixed64.MaxValue,out Fixed64 entry,out _))return false;
        Signed832 pValue=PackScaledCone832(p);
        bool contained;
        if(!hasEdge)
        {
            contained=crossesAxis
                ?TryGetScaledConeAxisPoint(triangle,frame,apex,axis,alignment,plane,false,out _,out _)
                :TryGetScaledConeStationaryPoint(triangle,frame,apex,axis,normal,normalSquare,alignment,plane,pValue,q,height,Fixed64.MaxValue,false,out _);
        }
        else
        {
            int sign=WideFiniteConeIntersection.GetBorrowedPolynomialSignAtRationalParameter(a,b,c,
                Signed832.ExtendValue(Signed192.Raw(entry)),Signed832.ExtendValue(MaximumParameter));
            Fixed64 lower=sign<0?Fixed64.FromRaw(entry.m_rawValue-1):entry;
            Fixed64 upper=sign>0?Fixed64.FromRaw(entry.m_rawValue+1):entry;
            contained=TryGetScaledConeStationaryPoint(triangle,frame,apex,axis,normal,normalSquare,alignment,plane,pValue,q,height,lower,false,out _)
                &&(lower==upper||TryGetScaledConeStationaryPoint(triangle,frame,apex,axis,normal,normalSquare,alignment,plane,pValue,q,height,upper,false,out _));
        }
        if(!contained||!TryGetScaledConeStationaryPoint(triangle,frame,apex,axis,normal,normalSquare,alignment,plane,pValue,q,height,entry,true,out point))return false;
        work.Set(Signed576.ExtendValue(Signed192.Raw(height)));ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(entry)),ratio);
        work.Set(Signed576.ExtendValue(MaximumParameter));
        axial=ContactQuadratic.RoundRatio(ratio,work,ReadOnlySpan<ulong>.Empty,0,height.m_rawValue);
        return true;
    }

    private static bool TryGetScaledConeAxisPoint(in FixedTriangle triangle,in ScaledRigidFrame3d frame,
        Vector3d apex,Vector3d axis,Signed832 alignment,Signed832 plane,bool materialize,
        out FixedPointAnchor point,out Fixed64 axial)
    {
        const int words=ScaledQueryWords;
        Span<ulong> values=stackalloc ulong[10*words];Span<int> signs=stackalloc int[10];
        var x=ContactQuadratic.At(values,signs,0,words);var y=ContactQuadratic.At(values,signs,1,words);
        var z=ContactQuadratic.At(values,signs,2,words);var d=ContactQuadratic.At(values,signs,3,words);
        var work=ContactQuadratic.At(values,signs,4,words);
        work.Set(plane);ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(axis.X)),x);
        ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(axis.Y)),y);
        ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(axis.Z)),z);
        work.Set(alignment);ContactQuadratic.Scale(work,Signed576.ExtendValue(frame.Denominator),d);
        if(d.Signs[0]<0){d.MultiplySign(-1);x.MultiplySign(-1);y.MultiplySign(-1);z.MultiplySign(-1);}
        point=default;axial=default;
        if(!ContainsScaledConeFacePoint(triangle,frame,apex,x,y,z,d))return false;
        if(!materialize)return true;
        if(!QuadraticPointMaterialization.TryGetPointInFrame(apex,FixedQuaternion.Identity,x,y,z,d,ReadOnlySpan<ulong>.Empty,apex,out Vector3d local))return false;
        point=new FixedPointAnchor(apex,FixedQuaternion.Identity,local);
        work.Set(plane);ContactQuadratic.Scale(work,Signed576.ExtendValue(Scale),x);
        if(alignment.Sign<0)x.MultiplySign(-1);
        axial=ContactQuadratic.RoundRatio(x,d,ReadOnlySpan<ulong>.Empty,0,long.MaxValue);
        return true;
    }

    private static bool TryGetScaledConeStationaryPoint(in FixedTriangle triangle,in ScaledRigidFrame3d frame,
        Vector3d apex,Vector3d axis,in WideDirection3d normal,Signed832 normalSquare,Signed832 alignment,
        Signed832 plane,Signed832 p,Signed832 q,Fixed64 height,Fixed64 parameter,bool materialize,out FixedPointAnchor point)
    {
        const int words=ScaledQueryWords;
        Span<ulong> values=stackalloc ulong[16*words];Span<int> signs=stackalloc int[16];
        var x=ContactQuadratic.At(values,signs,0,words);var y=ContactQuadratic.At(values,signs,1,words);
        var z=ContactQuadratic.At(values,signs,2,words);var d=ContactQuadratic.At(values,signs,3,words);
        var work=ContactQuadratic.At(values,signs,4,words);var term=ContactQuadratic.At(values,signs,5,words);
        var w=ContactQuadratic.At(values,signs,6,words);var v=ContactQuadratic.At(values,signs,7,words);
        work.Set(p);ContactQuadratic.Scale(work,Signed576.ExtendValue(frame.Denominator),term);
        ContactQuadratic.Scale(term,Signed576.ExtendValue(Scale),work);ContactQuadratic.Scale(work,Signed576.ExtendValue(MaximumParameter),d);
        for(int coordinate=0;coordinate<3;coordinate++)
        {
            var output=coordinate==0?x:coordinate==1?y:z;
            Signed576 n=coordinate==0?normal.X:coordinate==1?normal.Y:normal.Z;
            work.Set(normalSquare);ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(axis[coordinate])),w);
            work.Set(alignment);ContactQuadratic.Scale(work,n,term);w.Add(term,-1);
            work.Set(q);ContactQuadratic.Scale(work,n,v);
            work.Set(alignment);ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(axis[coordinate])),term);v.Add(term,-1);
            ContactQuadratic.Scale(w,q,work);ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(height)),term);
            ContactQuadratic.Scale(term,Signed576.ExtendValue(frame.Denominator),work);ContactQuadratic.Scale(work,Signed576.ExtendValue(Signed192.Raw(parameter)),output);
            ContactQuadratic.Scale(v,plane,work);ContactQuadratic.Scale(work,Signed576.ExtendValue(Scale),term);
            ContactQuadratic.Scale(term,Signed576.ExtendValue(MaximumParameter),work);output.Add(work);
        }
        point=default;
        if(!ContainsScaledConeFacePoint(triangle,frame,apex,x,y,z,d))return false;
        if(!materialize)return true;
        if(!QuadraticPointMaterialization.TryGetPointInFrame(apex,FixedQuaternion.Identity,x,y,z,d,ReadOnlySpan<ulong>.Empty,apex,out Vector3d local))return false;
        point=new FixedPointAnchor(apex,FixedQuaternion.Identity,local);return true;
    }

    private static bool ContainsScaledConeFacePoint(in FixedTriangle triangle,in ScaledRigidFrame3d frame,Vector3d apex,
        scoped ContactQuadratic x,scoped ContactQuadratic y,scoped ContactQuadratic z,scoped ContactQuadratic denominator)
    {
        const int words=ScaledQueryWords;
        Span<ulong> values=stackalloc ulong[8*words];Span<int> signs=stackalloc int[8];
        var sum=ContactQuadratic.At(values,signs,0,words);var delta=ContactQuadratic.At(values,signs,1,words);
        var term=ContactQuadratic.At(values,signs,2,words);var product=ContactQuadratic.At(values,signs,3,words);
        WideDirection3d normal=WideTriangleRelations.GetScaledTriangleLocalNormal(triangle,frame);
        for(int edge=0;edge<3;edge++)
        {
            FixedSegment segment=triangle.GetEdge(edge);
            WideDirection3d inward=WideDirection3d.Cross(normal,WideDirection3d.GetLocalEdge(frame,segment.Start,segment.End)).Transform(frame.GeometryBasis);
            WideDirection3d start=GetScaledConeRelativePoint(frame.GetPoint(segment.Start),apex);sum.Clear();
            for(int coordinate=0;coordinate<3;coordinate++)
            {
                var point=coordinate==0?x:coordinate==1?y:z;
                ContactQuadratic.Scale(point,Signed576.ExtendValue(frame.Denominator),delta);
                ContactQuadratic.Scale(denominator,coordinate==0?start.X:coordinate==1?start.Y:start.Z,term);delta.Add(term,-1);
                ContactQuadratic.Scale(delta,coordinate==0?inward.X:coordinate==1?inward.Y:inward.Z,product);sum.Add(product);
            }
            if(sum.Signs[0]<0)return false;
        }
        return true;
    }

    private static Signed832 PackScaledCone832(scoped ContactQuadratic value)
    {
        // Both callers pack nonnegative values: the positive axial cap and
        // the Gram minor |normal|²|axis|²-(normal.axis)².
        System.Diagnostics.Debug.Assert(value.Signs[0]>=0);
        Span<ulong> data=stackalloc ulong[13];data.Clear();value.Rational[..WideArithmetic.GetActiveMagnitudeLength(value.Rational)].CopyTo(data);
        return new Signed832(data[12],data[11],data[10],data[9],data[8],data[7],data[6],data[5],data[4],data[3],data[2],data[1],data[0]);
    }
}
