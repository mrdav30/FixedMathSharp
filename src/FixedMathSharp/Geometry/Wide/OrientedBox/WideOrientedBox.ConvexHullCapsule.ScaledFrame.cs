//=======================================================================
// WideOrientedBox.ConvexHullCapsule.ScaledFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Retains authored affine hull points through the existing finite capsule SAT candidates.</content>
internal static partial class WideOrientedBox
{
    // Bounded physical reach is at most 4096. A common source/core point needs
    // <360 bits, degree-three radial axes <1080, and their squares <2160.
    // All candidate products stay in borrowed banks; no scalar vertex is made.
    private const int ScaledHullCapsuleWords = 64;

    private readonly ref struct ScaledHullCapsuleFrame
    {
        internal readonly ScaledRigidFrame3d Frame;
        internal readonly ReadOnlySpan<Vector3d> Points;
        internal readonly ReadOnlySpan<int> Triangles, Edges;
        internal readonly Vector3d Center;
        internal readonly RationalPoint3d First, Second;
        internal readonly Signed576 Denominator;
        private readonly Signed192 coreFactor;
        internal ScaledHullCapsuleFrame(in ScaledRigidFrame3d frame, ReadOnlySpan<Vector3d> points,
            ReadOnlySpan<int> triangles, ReadOnlySpan<int> edges, Vector3d center,
            in RationalPoint3d first, in RationalPoint3d second)
        {
            Frame = frame; Points = points; Triangles = triangles; Edges = edges; Center = center;
            First = first; Second = second;
            coreFactor = WideArithmetic.TruncateSigned320By64(first.Denominator);
            Denominator = WideArithmetic.MultiplySigned320(frame.Denominator, coreFactor);
        }
        internal WideDirection3d Point(int index)
        {
            RationalPoint3d point = Frame.GetPoint(Points[index]);
            return Relative(point, coreFactor);
        }
        internal WideDirection3d Endpoint(bool second) => Relative(second ? Second : First, Frame.RotationDenominator);
        private WideDirection3d Relative(in RationalPoint3d point, Signed192 factor) => new(
            Coordinate(point.X, point.Denominator, Center.X, factor),
            Coordinate(point.Y, point.Denominator, Center.Y, factor),
            Coordinate(point.Z, point.Denominator, Center.Z, factor));
        private static Signed576 Coordinate(Signed320 numerator, Signed320 denominator, Fixed64 center, Signed192 factor) =>
            WideArithmetic.MultiplySigned576(WideArithmetic.SubtractSigned576(Signed576.ExtendValue(numerator),
                WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(center))), factor);
    }

    internal static bool TryGetConvexHullCenteredCapsuleContact(in ScaledRigidFrame3d frame, Vector3d hullAnchorOrigin,
        ReadOnlySpan<Vector3d> points, ReadOnlySpan<int> triangles, ReadOnlySpan<int> edges,
        Vector3d center, FixedQuaternion rotation, Vector3d localAxis, Fixed64 length, Fixed64 radius,
        out FixedContactAnchors contact)
    {
        if (frame.IsRigid) return TryGetConvexHullCenteredCapsuleContact(frame.RigidOrigin, frame.Rotation,
            points, triangles, edges, center, rotation, localAxis, length, radius, out contact);
        contact = default;
        FixedPointAnchor first = WideGeometry.GetCenteredCapsuleSupportAnchor(center, rotation, localAxis,
            length, Fixed64.Zero, Vector3d.Zero, -1);
        FixedPointAnchor second = WideGeometry.GetCenteredCapsuleSupportAnchor(center, rotation, localAxis,
            length, Fixed64.Zero, Vector3d.Zero, 1);
        var geometry = new ScaledHullCapsuleFrame(frame, points, triangles, edges, center,
            RationalPoint3d.FromAnchor(first), RationalPoint3d.FromAnchor(second));
        const int words = ScaledHullCapsuleWords;
        Span<ulong> values = stackalloc ulong[14 * words]; Span<int> signs = stackalloc int[14];
        var x = ContactQuadratic.At(values, signs, 0, words);
        var y = ContactQuadratic.At(values, signs, 1, words);
        var z = ContactQuadratic.At(values, signs, 2, words);
        var rational = ContactQuadratic.At(values, signs, 3, words);
        var square = ContactQuadratic.At(values, signs, 4, words);
        var bestRational = ContactQuadratic.At(values, signs, 5, words);
        var bestSquare = ContactQuadratic.At(values, signs, 6, words);
        if (!FindScaledHullCapsuleCandidate(geometry, radius, false, x, y, z, rational, square,
                bestRational, bestSquare, out HullCapsuleCandidate best)) return false;
        _ = EvaluateScaledHullCapsuleCandidate(geometry, best, radius, x, y, z, rational, square, out bool negate);
        if (negate) { x.MultiplySign(-1); y.MultiplySign(-1); z.MultiplySign(-1); }
        Vector3d normal = NormalizeScaledHullAxis(x, y, z);
        SetScaledHullAxisDot(x, y, z, Subtract(geometry.Endpoint(true), geometry.Endpoint(false)), bestRational, bestSquare);
        int axialSign = -bestRational.Signs[0];
        FixedPointAnchor capsule = WideGeometry.GetCenteredCapsuleSupportAnchor(center, rotation, localAxis,
            length, radius, -normal, axialSign);
        FixedPointAnchor hull = GetScaledHullCapsuleSupportAnchor(geometry, x, y, z, hullAnchorOrigin,
            center, rotation, localAxis, length, radius, normal, capsule, out capsule);
        GetHullCapsuleDepth(rational.Rational, rational.Signs[0], square.Rational,
            geometry.Denominator, radius, out Fixed64 depth, out bool clamped);
        contact = new FixedContactAnchors(hull, capsule, normal, depth, clamped);
        return true;
    }

    internal static bool DoesConvexHullCenteredCapsuleStrictlyOverlap(in ScaledRigidFrame3d frame,
        ReadOnlySpan<Vector3d> points, ReadOnlySpan<int> triangles, ReadOnlySpan<int> edges,
        Vector3d center, FixedQuaternion rotation, Vector3d localAxis, Fixed64 length, Fixed64 radius)
    {
        if (frame.IsRigid) return DoesConvexHullCenteredCapsuleStrictlyOverlap(frame.RigidOrigin, frame.Rotation,
            points, triangles, edges, center, rotation, localAxis, length, radius);
        var first = RationalPoint3d.FromAnchor(WideGeometry.GetCenteredCapsuleSupportAnchor(center, rotation,
            localAxis, length, Fixed64.Zero, Vector3d.Zero, -1));
        var second = RationalPoint3d.FromAnchor(WideGeometry.GetCenteredCapsuleSupportAnchor(center, rotation,
            localAxis, length, Fixed64.Zero, Vector3d.Zero, 1));
        var geometry = new ScaledHullCapsuleFrame(frame, points, triangles, edges, center, first, second);
        const int words = ScaledHullCapsuleWords;
        Span<ulong> values = stackalloc ulong[14 * words]; Span<int> signs = stackalloc int[14];
        return FindScaledHullCapsuleCandidate(geometry, radius, true,
            ContactQuadratic.At(values, signs, 0, words), ContactQuadratic.At(values, signs, 1, words),
            ContactQuadratic.At(values, signs, 2, words), ContactQuadratic.At(values, signs, 3, words),
            ContactQuadratic.At(values, signs, 4, words), ContactQuadratic.At(values, signs, 5, words),
            ContactQuadratic.At(values, signs, 6, words), out _);
    }

    private static FixedPointAnchor GetScaledHullCapsuleSupportAnchor(scoped in ScaledHullCapsuleFrame geometry,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, Vector3d hullOrigin,
        Vector3d center, FixedQuaternion rotation, Vector3d localAxis, Fixed64 length, Fixed64 radius,
        Vector3d normal, in FixedPointAnchor target, out FixedPointAnchor matchedCapsule)
    {
        matchedCapsule = target;
        const int words = ScaledHullCapsuleWords;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var support = ContactQuadratic.At(values, signs, 0, words);
        var projection = ContactQuadratic.At(values, signs, 1, words);
        var term = ContactQuadratic.At(values, signs, 2, words);
        var coreProjection = ContactQuadratic.At(values, signs, 3, words);
        SetScaledHullAxisDot(x, y, z, geometry.Point(0), support, term);
        for (int index = 1; index < geometry.Points.Length; index++)
        {
            SetScaledHullAxisDot(x, y, z, geometry.Point(index), projection, term);
            if (CompareScaledHullValues(projection, support) > 0) projection.CopyTo(support);
        }
        SetScaledHullAxisDot(x, y, z, Subtract(geometry.Endpoint(true), geometry.Endpoint(false)), coreProjection, term);
        bool matchCore = coreProjection.Signs[0] == 0 && length > Fixed64.Zero;
        RationalPoint3d targetPoint = RationalPoint3d.FromAnchor(target);
        // A finite support-face projection certifies the entire support set.
        for (int index = 0; index < geometry.Triangles.Length; index += 3)
        {
            int ia = geometry.Triangles[index], ib = geometry.Triangles[index + 1], ic = geometry.Triangles[index + 2];
            if (!IsScaledHullCapsuleSupport(geometry, ia, x,y,z,support,projection,term)
                || !IsScaledHullCapsuleSupport(geometry, ib, x,y,z,support,projection,term)
                || !IsScaledHullCapsuleSupport(geometry, ic, x,y,z,support,projection,term)) continue;
            var triangle = new FixedTriangle(geometry.Points[ia],geometry.Points[ib],geometry.Points[ic]);
            if (WideTriangleRelations.TryGetProjectedPointAnchor(triangle, geometry.Frame, targetPoint, hullOrigin,
                    out FixedPointAnchor face)) return face;
        }
        Span<ulong> bestValues = stackalloc ulong[WideFiniteAxisIntersection.ScaledAxisTriangleCandidate.StorageWords];
        Span<int> bestSigns = stackalloc int[WideFiniteAxisIntersection.ScaledAxisTriangleCandidate.SignCount];
        var best = new WideFiniteAxisIntersection.ScaledAxisTriangleCandidate(bestValues,bestSigns);
        RationalPoint3d first = matchCore ? geometry.First : targetPoint, second = matchCore ? geometry.Second : targetPoint;
        // Faces, edges and vertices form the same finite support set as the
        // rigid owner. The shared closest-feature kernel retains both witnesses.
        for (int kind = 0; kind < 3; kind++)
        {
            int count = kind == 0 ? geometry.Triangles.Length/3 : kind == 1 ? geometry.Edges.Length/2 : geometry.Points.Length;
            for(int index = 0; index < count; index++)
            {
                int ia = kind == 0 ? geometry.Triangles[3*index] : kind == 1 ? geometry.Edges[2*index] : index;
                int ib = kind == 0 ? geometry.Triangles[3*index+1] : kind == 1 ? geometry.Edges[2*index+1] : ia;
                int ic = kind == 0 ? geometry.Triangles[3*index+2] : ib;
                if(!IsScaledHullCapsuleSupport(geometry,ia,x,y,z,support,projection,term)
                    ||!IsScaledHullCapsuleSupport(geometry,ib,x,y,z,support,projection,term)
                    ||!IsScaledHullCapsuleSupport(geometry,ic,x,y,z,support,projection,term)) continue;
                var triangle = new FixedTriangle(geometry.Points[ia],geometry.Points[ib],geometry.Points[ic]);
                WideFiniteAxisIntersection.GetClosestScaledAxisTriangleCandidate(triangle,geometry.Frame,
                    first,second,center,ref best);
            }
        }
        System.Diagnostics.Debug.Assert(best.HasValue);
        bool represented = QuadraticPointMaterialization.TryGetPointInFrame(center,FixedQuaternion.Identity,
            best.TriangleCoordinate(0),best.TriangleCoordinate(1),best.TriangleCoordinate(2),best.Denominator,
            ReadOnlySpan<ulong>.Empty,hullOrigin,out Vector3d hullPoint);
        System.Diagnostics.Debug.Assert(represented);
        if(matchCore)
        {
            // Add radial support before the only scalar rounding, preserving an
            // odd endpoint or fractional projected core coordinate.
            int witnessWords=best.Denominator.FieldWords+2;
            Span<ulong> material=stackalloc ulong[10*witnessWords];Span<int> materialSigns=stackalloc int[10];
            var a=ContactQuadratic.At(material,materialSigns,0,witnessWords);
            var b=ContactQuadratic.At(material,materialSigns,1,witnessWords);
            var c=ContactQuadratic.At(material,materialSigns,2,witnessWords);
            var d=ContactQuadratic.At(material,materialSigns,3,witnessWords);
            var radial=ContactQuadratic.At(material,materialSigns,4,witnessWords);
            ContactQuadratic.Scale(best.Denominator,Signed576.ExtendValue(Signed192.Raw(Fixed64.One)),d);
            for(int axis=0;axis<3;axis++)
            {
                var output=axis==0?a:axis==1?b:c;
                ContactQuadratic.Scale(best.AxisCoordinate(axis),Signed576.ExtendValue(Signed192.Raw(Fixed64.One)),output);
                ContactQuadratic.Scale(best.Denominator,Signed576.ExtendValue(Fixed64.GetExactRawProduct(normal[axis],radius)),radial);
                output.Add(radial,-1);
            }
            represented=QuadraticPointMaterialization.TryGetPointInFrame(center,FixedQuaternion.Identity,a,b,c,d,
                ReadOnlySpan<ulong>.Empty,center,out Vector3d capsulePoint);
            System.Diagnostics.Debug.Assert(represented);
            matchedCapsule=new FixedPointAnchor(center,FixedQuaternion.Identity,capsulePoint);
        }
        return new FixedPointAnchor(hullOrigin,FixedQuaternion.Identity,hullPoint);
    }

    private static bool IsScaledHullCapsuleSupport(scoped in ScaledHullCapsuleFrame geometry,int index,
        scoped ContactQuadratic x,scoped ContactQuadratic y,scoped ContactQuadratic z,
        scoped ContactQuadratic support,scoped ContactQuadratic projection,scoped ContactQuadratic term)
    { SetScaledHullAxisDot(x,y,z,geometry.Point(index),projection,term);return CompareScaledHullValues(projection,support)==0; }

    private static bool FindScaledHullCapsuleCandidate(scoped in ScaledHullCapsuleFrame geometry, Fixed64 radius,
        bool strict, scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z,
        scoped ContactQuadratic rational, scoped ContactQuadratic square,
        scoped ContactQuadratic bestRational, scoped ContactQuadratic bestSquare, out HullCapsuleCandidate best)
    {
        bool found = false; best = default;
        for (int kind = 0; kind < 4; kind++)
        {
            int count = kind == 0 ? geometry.Triangles.Length / 3
                : kind == 2 ? geometry.Points.Length : geometry.Edges.Length / 2;
            for (int feature = 0; feature < count; feature++)
            for (int endpoint = kind == 3 ? -1 : 0; endpoint <= (kind == 3 ? 1 : 0); endpoint += 2)
            {
                var candidate = new HullCapsuleCandidate((HullCapsuleCandidateKind)kind,
                    kind == 0 ? feature * 3 : kind == 2 ? feature : feature * 2, endpoint);
                int status = EvaluateScaledHullCapsuleCandidate(geometry, candidate, radius,
                    x, y, z, rational, square, out _);
                if (status < 0 || strict && status > 0 && GetScaledHullCapsuleOverlapSign(
                        rational, square, geometry.Denominator, radius) == 0) return false;
                if (status == 0) continue;
                if (!found || !strict && WideArithmetic.CompareSignedNormalizedMagnitudes(rational.Rational,
                    rational.Signs[0], square.Rational, bestRational.Rational, bestRational.Signs[0], bestSquare.Rational) < 0)
                {
                    found = true; best = candidate; rational.CopyTo(bestRational); square.CopyTo(bestSquare);
                }
            }
        }
        return found;
    }

    private static int EvaluateScaledHullCapsuleCandidate(scoped in ScaledHullCapsuleFrame geometry,
        HullCapsuleCandidate candidate, Fixed64 radius, scoped ContactQuadratic x, scoped ContactQuadratic y,
        scoped ContactQuadratic z, scoped ContactQuadratic rational, scoped ContactQuadratic square, out bool negate)
    {
        BuildScaledHullCapsuleAxis(geometry, candidate, x, y, z);
        negate = false; rational.Clear(); square.Clear();
        if (x.Signs[0] == 0 && y.Signs[0] == 0 && z.Signs[0] == 0) return 0;
        const int words = ScaledHullCapsuleWords;
        Span<ulong> values = stackalloc ulong[12 * words]; Span<int> signs = stackalloc int[12];
        var minimum = ContactQuadratic.At(values, signs, 0, words);
        var maximum = ContactQuadratic.At(values, signs, 1, words);
        var projection = ContactQuadratic.At(values, signs, 2, words);
        var term = ContactQuadratic.At(values, signs, 3, words);
        var positive = ContactQuadratic.At(values, signs, 4, words);
        var negative = ContactQuadratic.At(values, signs, 5, words);
        SetScaledHullAxisDot(x, y, z, geometry.Point(0), minimum, term); minimum.CopyTo(maximum);
        for (int index = 1; index < geometry.Points.Length; index++)
        {
            SetScaledHullAxisDot(x, y, z, geometry.Point(index), projection, term);
            if (CompareScaledHullValues(projection, minimum) < 0) projection.CopyTo(minimum);
            if (CompareScaledHullValues(projection, maximum) > 0) projection.CopyTo(maximum);
        }
        SetScaledHullAxisDot(x, y, z, geometry.Endpoint(false), positive, term);
        SetScaledHullAxisDot(x, y, z, geometry.Endpoint(true), negative, term);
        if (CompareScaledHullValues(positive, negative) > 0)
        { positive.CopyTo(projection); negative.CopyTo(positive); projection.CopyTo(negative); }
        maximum.CopyTo(rational); rational.Add(positive, -1);
        negative.Add(minimum, -1);
        if (CompareScaledHullValues(negative, rational) < 0) { negative.CopyTo(rational); negate = true; }
        ContactQuadratic.Multiply(x, x, ReadOnlySpan<ulong>.Empty, square);
        ContactQuadratic.Multiply(y, y, ReadOnlySpan<ulong>.Empty, term); square.Add(term);
        ContactQuadratic.Multiply(z, z, ReadOnlySpan<ulong>.Empty, term); square.Add(term);
        return GetScaledHullCapsuleOverlapSign(rational, square, geometry.Denominator, radius) < 0 ? -1 : 1;
    }

    private static int GetScaledHullCapsuleOverlapSign(scoped ContactQuadratic rational, scoped ContactQuadratic square,
        Signed576 denominator, Fixed64 radius)
    {
        Span<ulong> coefficient = stackalloc ulong[13];
        Signed832 radial = WideArithmetic.MultiplySigned576ToSigned832(denominator, Signed576.ExtendValue(Signed192.Raw(radius)));
        WideArithmetic.GetMagnitude(radial, coefficient);
        return WideArithmetic.GetSignedMagnitudeAndSquareRootSign(rational.Rational, rational.Signs[0],
            coefficient, radial.Sign, square.Rational);
    }

    private static void BuildScaledHullCapsuleAxis(scoped in ScaledHullCapsuleFrame geometry,
        HullCapsuleCandidate candidate, scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z)
    {
        WideDirection3d first = geometry.Endpoint(false), core = Subtract(geometry.Endpoint(true), first);
        WideDirection3d edge, offset;
        if (candidate.Kind == HullCapsuleCandidateKind.Face)
        {
            WideDirection3d a = geometry.Point(geometry.Triangles[candidate.Index]);
            SetScaledHullCross(Subtract(geometry.Point(geometry.Triangles[candidate.Index + 1]), a),
                Subtract(geometry.Point(geometry.Triangles[candidate.Index + 2]), a), x, y, z); return;
        }
        if (candidate.Kind == HullCapsuleCandidateKind.VertexCore)
        { edge = core; offset = Subtract(geometry.Point(candidate.Index), first); }
        else
        {
            WideDirection3d start = geometry.Point(geometry.Edges[candidate.Index]);
            edge = Subtract(geometry.Point(geometry.Edges[candidate.Index + 1]), start);
            if (candidate.Kind == HullCapsuleCandidateKind.EdgeCross) { SetScaledHullCross(edge, core, x, y, z); return; }
            offset = Subtract(geometry.Endpoint(candidate.EndpointSign > 0), start);
        }
        const int words = ScaledHullCapsuleWords;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var length = ContactQuadratic.At(values, signs, 0, words);
        var projection = ContactQuadratic.At(values, signs, 1, words);
        var term = ContactQuadratic.At(values, signs, 2, words);
        var coordinate = ContactQuadratic.At(values, signs, 3, words);
        SetScaledHullVectorDot(edge, edge, length, term); SetScaledHullVectorDot(offset, edge, projection, term);
        if (length.Signs[0] == 0 || projection.Signs[0] <= 0) { x.Set(offset.X); y.Set(offset.Y); z.Set(offset.Z); return; }
        if (CompareScaledHullValues(projection, length) >= 0)
        { offset = Subtract(offset, edge); x.Set(offset.X); y.Set(offset.Y); z.Set(offset.Z); return; }
        for (int axis = 0; axis < 3; axis++)
        {
            var output = axis == 0 ? x : axis == 1 ? y : z;
            ContactQuadratic.Scale(length, axis == 0 ? offset.X : axis == 1 ? offset.Y : offset.Z, output);
            ContactQuadratic.Scale(projection, axis == 0 ? edge.X : axis == 1 ? edge.Y : edge.Z, coordinate);
            output.Add(coordinate, -1);
        }
    }

    private static void SetScaledHullCross(in WideDirection3d first, in WideDirection3d second,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z)
    {
        const int words = ScaledHullCapsuleWords;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var value = ContactQuadratic.At(values, signs, 0, words); var term = ContactQuadratic.At(values, signs, 1, words);
        for (int axis = 0; axis < 3; axis++)
        {
            var output = axis == 0 ? x : axis == 1 ? y : z;
            Signed576 a = axis == 0 ? first.Y : axis == 1 ? first.Z : first.X;
            Signed576 b = axis == 0 ? second.Z : axis == 1 ? second.X : second.Y;
            value.Set(a); ContactQuadratic.Scale(value, b, output);
            a = axis == 0 ? first.Z : axis == 1 ? first.X : first.Y;
            b = axis == 0 ? second.Y : axis == 1 ? second.Z : second.X;
            value.Set(a); ContactQuadratic.Scale(value, b, term); output.Add(term, -1);
        }
    }

    private static WideDirection3d Subtract(in WideDirection3d a, in WideDirection3d b) => new(
        WideArithmetic.SubtractSigned576(a.X,b.X), WideArithmetic.SubtractSigned576(a.Y,b.Y), WideArithmetic.SubtractSigned576(a.Z,b.Z));
    private static int CompareScaledHullValues(scoped ContactQuadratic a, scoped ContactQuadratic b) =>
        CompareHullCapsuleSignedMagnitudes(a.Rational, a.Signs[0], b.Rational, b.Signs[0]);
    private static void SetScaledHullVectorDot(in WideDirection3d a, in WideDirection3d b,
        scoped ContactQuadratic result, scoped ContactQuadratic term)
    {
        result.Clear(); term.Set(a.X); ContactQuadratic.Scale(term,b.X,result);
        term.Set(a.Y); Span<ulong> v=stackalloc ulong[2*ScaledHullCapsuleWords]; Span<int> s=stackalloc int[2];
        var product=ContactQuadratic.At(v,s,0,ScaledHullCapsuleWords); ContactQuadratic.Scale(term,b.Y,product); result.Add(product);
        term.Set(a.Z); ContactQuadratic.Scale(term,b.Z,product); result.Add(product);
    }
    private static void SetScaledHullAxisDot(scoped ContactQuadratic x, scoped ContactQuadratic y,
        scoped ContactQuadratic z, in WideDirection3d point, scoped ContactQuadratic result, scoped ContactQuadratic term)
    {
        ContactQuadratic.Scale(x,point.X,result); ContactQuadratic.Scale(y,point.Y,term); result.Add(term);
        ContactQuadratic.Scale(z,point.Z,term); result.Add(term);
    }
    private static Vector3d NormalizeScaledHullAxis(scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z)
    {
        ReadOnlySpan<ulong> maximum = WideArithmetic.CompareMagnitudeEqualLength(x.Rational,y.Rational)>=0?x.Rational:y.Rational;
        if(WideArithmetic.CompareMagnitudeEqualLength(z.Rational,maximum)>0) maximum=z.Rational;
        Span<ulong> numerator=stackalloc ulong[ScaledHullCapsuleWords+1]; Vector3d result=default;
        for(int axis=0;axis<3;axis++)
        {
            var value=axis==0?x:axis==1?y:z; numerator.Clear(); int sign=0;
            WideArithmetic.AddShiftedSignedMagnitude(value.Rational,1,32,numerator,ref sign);
            _=Fixed64.TryGetSignedRawRatio(numerator,maximum,value.Signs[0]<0,out Fixed64 coordinate); result[axis]=coordinate;
        }
        return Vector3d.GetScaleNormalized(result);
    }
}
