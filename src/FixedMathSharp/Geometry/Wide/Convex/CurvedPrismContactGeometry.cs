//=======================================================================
// CurvedPrismContactGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Exact native-yaw prism geometry in the curved primitive's authored quaternion frame.</summary>
internal readonly ref struct CurvedPrismContactGeometry
{
    internal readonly WideRationalBasis3d WorldBasis;
    internal readonly WideAxis3 Axis;
    internal readonly Signed320 SquaredAxis, Radius;
    internal readonly Signed192 HalfFactor, RawScale;
    internal readonly WideDirection3d Half;
    internal readonly int ValueShift;
    internal readonly bool IsCone;
    internal readonly Fixed64 PrismHalfThickness, HeightValue, RadiusValue;
    private readonly WideConvex2dRelations.RotationFrame2d _yaw;
    private readonly ReadOnlySpan<Vector2d> _offsets;
    private readonly Signed192 _relativeX, _relativeY, _relativeZ, _thickness;
    private readonly Signed192 _coordinateDivisor;
    private readonly ReadOnlySpan<WideDirection3d> _vertices;

    internal int VertexCount => 2 * _offsets.Length;
    internal int EdgeCount => 3 * _offsets.Length;
    internal int PlaneCount => 3 + _offsets.Length;

    internal CurvedPrismContactGeometry(Vector3d center, FixedQuaternion rotation,
        Vector3d localAxis, Fixed64 length, Fixed64 radius, Vector3d prismOrigin,
        Fixed64 prismYaw, ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness, bool isCone,
        Span<WideDirection3d> vertexStorage)
    {
        WorldBasis = new WideRationalBasis3d(rotation);
        Signed192 axisX = Signed192.Raw(localAxis.X), axisY = Signed192.Raw(localAxis.Y), axisZ = Signed192.Raw(localAxis.Z);
        Signed192 axisDivisor = WideArithmetic.GetGreatestCommonDivisor(
            WideArithmetic.GetGreatestCommonDivisor(axisX, axisY), axisZ);
        Axis = new WideAxis3(Signed320.ExtendValue(WideArithmetic.DivideExactSigned192(axisX, axisDivisor)),
            Signed320.ExtendValue(WideArithmetic.DivideExactSigned192(axisY, axisDivisor)),
            Signed320.ExtendValue(WideArithmetic.DivideExactSigned192(axisZ, axisDivisor)));
        SquaredAxis = Signed320.NarrowValue(Axis.SquaredLength);
        HalfFactor = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            WorldBasis.Denominator, Signed192.Raw(length)));
        HalfFactor = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(HalfFactor, axisDivisor));
        Half = new WideDirection3d(WideArithmetic.MultiplySigned320(Axis.X, HalfFactor),
            WideArithmetic.MultiplySigned320(Axis.Y, HalfFactor), WideArithmetic.MultiplySigned320(Axis.Z, HalfFactor));
        RawScale = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            WorldBasis.Denominator, Signed192.Raw(Fixed64.Two)));
        Radius = WideArithmetic.MultiplySigned192(RawScale, Signed192.Raw(radius));
        IsCone = isCone; PrismHalfThickness = halfThickness; HeightValue = length; RadiusValue = radius;
        _yaw = new WideConvex2dRelations.RotationFrame2d(prismYaw);
        _offsets = offsets;
        _relativeX = ScaleRaw(WideArithmetic.Difference(prismOrigin.X, center.X));
        _relativeY = ScaleRaw(WideArithmetic.Difference(prismOrigin.Y, center.Y));
        _relativeZ = ScaleRaw(WideArithmetic.Difference(prismOrigin.Z, center.Z));
        _thickness = ScaleRaw(Signed192.Raw(halfThickness));
        _coordinateDivisor = Signed192.Signed(1);
        _vertices = default;
        ValueShift = 0;

        // Native yaw vertices V have denominator Q. The exact quaternion
        // transpose gives P=2*B^T*V, S=2*D*Q, H=(D*Lraw)*A, R=S*Rraw.
        // Primitive Axis=A/g and T=D*Lraw*g preserve the authored half-axis;
        // no alignment quaternion or normalization changes its actual norm.
        // D<2^65 and the original |A|<2^33
        // give q<2^65, |P|<2^165, |H|,R<2^163 and S<2^99.
        // These bounds include full-domain origin differences and native yaw
        // offsets. T<2^161 and all coordinate scalars fit Signed192. Divide
        // S,T,R and every P by one positive content, retaining all ratios and
        // H=T*Axis exactly. T²*q is unchanged by primitive-axis reduction.
        Signed192 divisor = WideArithmetic.GetGreatestCommonDivisor(RawScale, HalfFactor);
        divisor = WideArithmetic.GetGreatestCommonDivisor(divisor, Signed192.NarrowValue(Radius));
        bool cache = VertexCount <= vertexStorage.Length;
        for (int index = 0; index < VertexCount; index++)
        {
            WideDirection3d vertex = GetNativeVertex(index);
            if (cache) vertexStorage[index] = vertex;
            divisor = WideArithmetic.GetGreatestCommonDivisor(divisor, Signed192.NarrowValue(vertex.X));
            divisor = WideArithmetic.GetGreatestCommonDivisor(divisor, Signed192.NarrowValue(vertex.Y));
            divisor = WideArithmetic.GetGreatestCommonDivisor(divisor, Signed192.NarrowValue(vertex.Z));
        }
        _coordinateDivisor = divisor;
        RawScale = WideArithmetic.DivideExactSigned192(RawScale, divisor);
        HalfFactor = WideArithmetic.DivideExactSigned192(HalfFactor, divisor);
        Radius = Signed320.ExtendValue(WideArithmetic.DivideExactSigned192(Signed192.NarrowValue(Radius), divisor));
        Half = new WideDirection3d(WideArithmetic.MultiplySigned320(Axis.X, HalfFactor),
            WideArithmetic.MultiplySigned320(Axis.Y, HalfFactor), WideArithmetic.MultiplySigned320(Axis.Z, HalfFactor));
        Span<ulong> magnitude = stackalloc ulong[9];
        int bits = Math.Max(Bits(Signed576.ExtendValue(Radius), magnitude),
            Math.Max(Bits(Half.X, magnitude), Math.Max(Bits(Half.Y, magnitude), Bits(Half.Z, magnitude))));
        for (int index = 0; index < VertexCount; index++)
        {
            WideDirection3d vertex = ReduceVertex(cache ? vertexStorage[index] : GetNativeVertex(index), divisor);
            if (cache) vertexStorage[index] = vertex;
            bits = Math.Max(bits, Math.Max(Bits(vertex.X, magnitude),
                Math.Max(Bits(vertex.Y, magnitude), Bits(vertex.Z, magnitude))));
        }
        // Borrow only a complete query-owned cache. Larger footprints stream
        // the same reduced vertices and never allocate or change feature order.
        if (cache) _vertices = vertexStorage[..VertexCount];
        // Every projected support residual is shorter than the sum of its
        // vertex, half-axis and radial terms. Four bits cover that sum and
        // three-coordinate length before squaring onto the retained value axis.
        ValueShift = 2 * (bits + 4);
    }

    /// <summary>Returns authored vertex 2*i at the bottom and 2*i+1 at the top.</summary>
    internal WideDirection3d Vertex(int index) => !_vertices.IsEmpty ? _vertices[index]
        : ReduceVertex(GetNativeVertex(index), _coordinateDivisor);

    internal Signed192 NativeRawScale => Signed192.NarrowProven(WideArithmetic.MultiplySigned192(RawScale, _coordinateDivisor));

    // Boundary cuts retain their original source coordinates; do not assume
    // the content of the original vertices also divides a new height-band point.
    internal WideDirection3d GetNativeSourcePoint(Vector3d point) =>
        GetNativeSourcePoint(new Vector2d(point.X, point.Z), ScaleRaw(Signed192.Raw(point.Y)));

    private WideDirection3d GetNativeVertex(int index) =>
        GetNativeSourcePoint(_offsets[index / 2], (index & 1) == 0 ? WideArithmetic.Negate(_thickness) : _thickness);

    private WideDirection3d GetNativeSourcePoint(Vector2d offset, Signed192 y)
    {
        WideConvex2dRelations.GetRotatedOffset(_yaw, offset, out Signed192 x, out Signed192 z);
        var native = new WideAxis3(Signed320.ExtendValue(WideArithmetic.AddSigned192(_relativeX, x)),
            Signed320.ExtendValue(WideArithmetic.AddSigned192(_relativeY, y)),
            Signed320.ExtendValue(WideArithmetic.AddSigned192(_relativeZ, z)));
        WideRigidProjection.GetLocalAxisProjections(native, WorldBasis, out Signed576 px, out Signed576 py, out Signed576 pz);
        return new WideDirection3d(WideArithmetic.AddSigned576(px, px),
            WideArithmetic.AddSigned576(py, py), WideArithmetic.AddSigned576(pz, pz));
    }

    internal Vector2d FootprintVertex(int index) => _offsets[index];

    /// <summary>Returns the bottom, top and vertical real edges of each footprint vertex.</summary>
    internal void Edge(int index, out int start, out int end)
    {
        int vertex = index / 3, kind = index % 3;
        int next = vertex + 1 == _offsets.Length ? 0 : vertex + 1;
        start = 2 * vertex + (kind == 1 ? 1 : 0);
        end = kind == 2 ? start + 1 : 2 * next + kind;
    }

    /// <summary>Returns exact native coordinate and footprint-edge plane normals.</summary>
    internal WideDirection3d Plane(int index)
    {
        WideDirection3d up = new(Signed576.ExtendValue(WorldBasis.Xy),
            Signed576.ExtendValue(WorldBasis.Yy), Signed576.ExtendValue(WorldBasis.Zy));
        if (index == 0) return new WideDirection3d(Signed576.ExtendValue(WorldBasis.Xx),
            Signed576.ExtendValue(WorldBasis.Yx), Signed576.ExtendValue(WorldBasis.Zx));
        if (index == 1) return up;
        if (index == 2) return new WideDirection3d(Signed576.ExtendValue(WorldBasis.Xz),
            Signed576.ExtendValue(WorldBasis.Yz), Signed576.ExtendValue(WorldBasis.Zz));
        Edge(3 * (index - 3), out int start, out int end);
        WideDirection3d a = Vertex(start), b = Vertex(end);
        WideDirection3d edge = new(WideArithmetic.SubtractSigned576(b.X, a.X),
            WideArithmetic.SubtractSigned576(b.Y, a.Y), WideArithmetic.SubtractSigned576(b.Z, a.Z));
        return WideDirection3d.Cross(edge, up);
    }

    /// <summary>Returns the two inward incident face normals of a proper real-edge support cone.</summary>
    internal bool TryGetEdgeNormalCone(int edge, out WideDirection3d first, out WideDirection3d last)
    {
        int vertex = edge / 3, kind = edge % 3;
        Edge(edge, out int start, out _);
        WideDirection3d point = Vertex(start);
        first = kind == 2 ? Plane(3 + (vertex == 0 ? _offsets.Length - 1 : vertex - 1)) : Plane(1);
        if (kind == 1) first = -first;
        last = Plane(3 + vertex);
        // The cap normal already minimizes at its bottom/top edge. Vertical
        // edges require both incident wall orientations to be certified.
        if ((kind == 2 && !TryOrientEdgePlane(point, ref first)) || !TryOrientEdgePlane(point, ref last)
            || WideDirection3d.AreParallel(first, last))
            return false;
        // Each endpoint supports the complete prism, so its positive span
        // does too. Primitive side normals are no larger than native
        // B^T*(-dz,0,dx), below 165 bits; their difference stays below 166.
        Span<Signed576> components = stackalloc Signed576[3];
        components[0] = first.X; components[1] = first.Y; components[2] = first.Z;
        WideArithmetic.ReduceCommonScale(components);
        first = new WideDirection3d(components[0], components[1], components[2]);
        components[0] = last.X; components[1] = last.Y; components[2] = last.Z;
        WideArithmetic.ReduceCommonScale(components);
        last = new WideDirection3d(components[0], components[1], components[2]);
        return true;
    }

    private bool TryOrientEdgePlane(in WideDirection3d point, ref WideDirection3d normal)
    {
        if (normal.IsZero) return false;
        Signed832 projection = WideDirection3d.Dot(normal, point);
        int minimumSign = 0, maximumSign = 0;
        // These incident wall normals are perpendicular to native Up, so
        // both extrusion levels have identical projections. One scan certifies
        // either orientation and rejects two-sided or entirely coplanar faces.
        for (int index = 0; index < VertexCount; index += 2)
        {
            int sign = WideArithmetic.SubtractSigned832(WideDirection3d.Dot(normal, Vertex(index)), projection).Sign;
            minimumSign = Math.Min(minimumSign, sign);
            maximumSign = Math.Max(maximumSign, sign);
        }
        int orientation = minimumSign + maximumSign;
        if (orientation == 0) return false;
        if (orientation < 0) normal = -normal;
        return true;
    }

    /// <summary>Returns the exact lower support-plane constant in common coordinates.</summary>
    internal Signed832 ProjectionMinimum(in WideDirection3d normal) => ProjectionMinimum(normal, out _);

    private Signed832 ProjectionMinimum(in WideDirection3d normal, out int vertex)
    {
        // Rational fan directions are below 504 bits and vertices below 165,
        // so dot products and their differences remain below 672 bits.
        // Every top-minus-bottom is the same positive multiple of native Up.
        // Choose that supporting level once; bottom preserves first-index ties.
        vertex = WideDirection3d.Dot(normal, Plane(1)).Sign < 0 ? 1 : 0;
        Signed832 minimum = WideDirection3d.Dot(normal, Vertex(vertex));
        for (int index = vertex + 2; index < VertexCount; index += 2)
        {
            Signed832 current = WideDirection3d.Dot(normal, Vertex(index));
            if (WideArithmetic.SubtractSigned832(current, minimum).Sign >= 0) continue;
            minimum = current; vertex = index;
        }
        return minimum;
    }

    /// <summary>
    /// Retains the first minimum projection under a seven-slot local normal
    /// (rational XYZ, radical XYZ, radicand). Result uses forty-word fields and
    /// must be disjoint from normal storage. The finite fan's rational fields
    /// are below 768 bits, radical fields below 512 and radicand below 832;
    /// vertex dots and their exact quadratic sign products fit forty words.
    /// </summary>
    internal void GetMinimum(ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns,
        ContactQuadratic result, out int vertexIndex)
    {
        Span<ulong> work = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic current = ContactQuadratic.At(work, signs, 0);
        ContactQuadratic difference = ContactQuadratic.At(work, signs, 1);
        DotNormal(Plane(1), normal, normalSigns, current);
        vertexIndex = current.Sign(Slot(normal, 6)) < 0 ? 1 : 0;
        DotNormal(Vertex(vertexIndex), normal, normalSigns, result);
        for (int index = vertexIndex + 2; index < VertexCount; index += 2)
        {
            DotNormal(Vertex(index), normal, normalSigns, current);
            current.CopyTo(difference); difference.Add(result, -1);
            if (difference.Sign(Slot(normal, 6)) >= 0) continue;
            current.CopyTo(result); vertexIndex = index;
        }
    }

    /// <summary>
    /// Encodes the complete support gap for a nonzero rational local direction, retaining
    /// that exact direction locally. Input and candidate storage are disjoint.
    /// Feature is -1 for bottom/base, +1 for top/apex, zero for a side/generator
    /// tie. A required vertex admits only its supporting normal directions.
    /// Directions fit Signed576; all support-gap products fit forty words.
    /// </summary>
    internal bool TryBuildCandidate(ReadOnlySpan<ulong> direction, ReadOnlySpan<int> directionSigns,
        Span<ulong> values, Span<int> signs, int requiredVertex, out int gapSign, out int vertex, out int feature)
    {
        var normal = new WideDirection3d(ReadDirectionComponent(Slot(direction, 0), directionSigns[0]),
            ReadDirectionComponent(Slot(direction, 1), directionSigns[1]),
            ReadDirectionComponent(Slot(direction, 2), directionSigns[2]));
        gapSign = 0; feature = 0; vertex = requiredVertex;
        if (requiredVertex >= 0)
        {
            // A required edge endpoint cannot support from the wrong extrusion level; zero admits both ties.
            int upSign = WideDirection3d.Dot(normal, Plane(1)).Sign;
            if ((requiredVertex & 1) == 0 ? upSign < 0 : upSign > 0) return false;
        }
        Signed832 minimum = ProjectionMinimum(normal, out vertex);
        // Edge-generated directions satisfy n.E=0. Exact support equality
        // at its start therefore admits the complete edge, including ties
        // whose first retained minimum happens to be another prism vertex.
        if (requiredVertex >= 0 && WideArithmetic.SubtractSigned832(
                WideDirection3d.Dot(normal, Vertex(requiredVertex)), minimum).Sign != 0)
            return false;
        gapSign = BuildPlaneCandidate(direction, directionSigns, normal, minimum, values, signs, out feature);
        return true;
    }

    // A retained native face uses its own known plane constant. In particular,
    // its opposite gap tests plane intersection, not the prism's opposite face.
    internal int BuildPlaneCandidate(ReadOnlySpan<ulong> direction, ReadOnlySpan<int> directionSigns,
        in WideDirection3d normal, Signed832 minimum, Span<ulong> values, Span<int> signs, out int feature)
    {
        values.Clear(); signs.Clear();
        direction.CopyTo(values); directionSigns.CopyTo(signs);
        Span<ulong> work = stackalloc ulong[14 * Words];
        Span<ulong> minimumMagnitude = Slot(work, 0), half = Slot(work, 1), metric = Slot(work, 2);
        Span<ulong> axial = Slot(work, 3), q = Slot(work, 4), radial = Slot(work, 5), radiusSquared = Slot(work, 6);
        Span<ulong> p = Slot(work, 7), root = Slot(work, 8), denominator = Slot(work, 9), temporary = Slot(work, 10);
        Span<ulong> product = Slot(work, 11), scale = Slot(work, 12), rational = Slot(work, 13);
        Import(minimum, minimumMagnitude);
        Signed832 axisProjection = WideDirection3d.Dot(normal, Axis);
        // Half=HalfFactor*Axis remains exact after both common-scale reductions.
        Signed832 halfProjection = WideArithmetic.MultiplySigned832(axisProjection, HalfFactor);
        Import(halfProjection, half);
        int halfSign = halfProjection.Sign;
        SumSquares(direction, metric);
        Import(axisProjection, axial);
        Import(SquaredAxis, q);
        WideArithmetic.MultiplyMagnitudes(q, metric, temporary);
        WideArithmetic.MultiplyMagnitudes(axial, axial, product);
        WideArithmetic.SubtractEqualMagnitudes(temporary, product, radial);
        Import(Radius, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, temporary, radiusSquared);
        bool apex = false;
        feature = halfSign;
        if (IsCone)
        {
            int comparison = -1;
            if (halfSign >= 0)
            {
                WideArithmetic.MultiplyMagnitudes(half, half, temporary);
                WideArithmetic.MultiplyMagnitudes(q, temporary, product);
                WideArithmetic.AddMagnitudeInto(product, product); WideArithmetic.AddMagnitudeInto(product, product);
                WideArithmetic.MultiplyMagnitudes(radiusSquared, radial, temporary);
                comparison = WideArithmetic.CompareMagnitudeEqualLength(product, temporary);
            }
            apex = comparison >= 0;
            feature = comparison == 0 ? 0 : apex ? 1 : -1;
        }
        half.CopyTo(rational);
        int rationalSign = IsCone ? (apex ? halfSign : -halfSign) : IsZero(half) ? 0 : 1;
        Add(minimumMagnitude, -minimum.Sign, rational, ref rationalSign);
        Import(Signed576.ExtendValue(RawScale), scale);
        WideArithmetic.MultiplyMagnitudes(scale, scale, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, metric, denominator);
        if (IsCone && apex)
        {
            rational.CopyTo(p); root.Clear();
        }
        else
        {
            // g=(q*T+sqrt(q*R²*(q*n²-(Axis.n)²)))/(q*S*|n|).
            // Retain the positive q scale exactly; no axis norm is rounded.
            WideArithmetic.MultiplyMagnitudes(q, rational, p);
            WideArithmetic.MultiplyMagnitudes(radiusSquared, radial, temporary);
            WideArithmetic.MultiplyMagnitudes(q, temporary, root);
            WideArithmetic.MultiplyMagnitudes(q, q, temporary);
            WideArithmetic.MultiplyMagnitudes(denominator, temporary, product); product.CopyTo(denominator);
        }
        int gapSign = BuildRadialCandidate(p, rationalSign, root, denominator, values, signs);
        return gapSign;
    }

    private static void DotNormal(in WideDirection3d point, ReadOnlySpan<ulong> normal,
        ReadOnlySpan<int> normalSigns, ContactQuadratic result)
    {
        Dot(point, normal[..(3 * Words)], normalSigns[..3], result.Rational, out int rationalSign);
        Dot(point, normal.Slice(3 * Words, 3 * Words), normalSigns.Slice(3, 3), result.Radical, out int radicalSign);
        result.Signs[0] = rationalSign; result.Signs[1] = radicalSign;
    }

    private static Signed192 ScaleRaw(Signed192 value) =>
        Signed192.NarrowProven(WideArithmetic.MultiplySigned192(value, Signed192.One));

    internal static Signed576 ReadDirectionComponent(ReadOnlySpan<ulong> magnitude, int sign)
    {
        // Rational fan components fit 504 bits; the remaining padded words
        // contain no information. Quadratic generator normals use GetMinimum.
        var value = new Signed576(magnitude[8], magnitude[7], magnitude[6], magnitude[5], magnitude[4],
            magnitude[3], magnitude[2], magnitude[1], magnitude[0]);
        return sign < 0 ? WideArithmetic.Negate(value) : value;
    }

    private static WideDirection3d ReduceVertex(in WideDirection3d vertex, Signed192 divisor) => new(
        Signed576.ExtendValue(WideArithmetic.DivideExactSigned192(Signed192.NarrowValue(vertex.X), divisor)),
        Signed576.ExtendValue(WideArithmetic.DivideExactSigned192(Signed192.NarrowValue(vertex.Y), divisor)),
        Signed576.ExtendValue(WideArithmetic.DivideExactSigned192(Signed192.NarrowValue(vertex.Z), divisor)));

    private static int Bits(Signed576 value, Span<ulong> magnitude)
    {
        WideArithmetic.GetMagnitude(value, magnitude);
        return WideArithmetic.GetMagnitudeBitLength(magnitude);
    }
}
