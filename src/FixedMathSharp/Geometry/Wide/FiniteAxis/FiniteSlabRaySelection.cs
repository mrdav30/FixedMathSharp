//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

internal readonly struct FiniteSlabRaySelectionMetadata
{
    internal readonly bool HasValue, ParameterIsDepth, HasCompactCell, IsRational;
    internal readonly int Ordinal, RootCount, CoefficientCount, Orientation, DenominatorShift;
    /// <summary>
    /// Bit i marks an active authored domain edge i that generated this event.
    /// Zero marks a shape-intrinsic event, which remains intrinsic when its
    /// source point happens to coincide with a covered mesh seam.
    /// </summary>
    internal readonly int DomainEdgeMask;
    internal readonly FixedTriangle Triangle;
    internal FiniteSlabRaySelectionMetadata(bool hasValue, bool parameterIsDepth, int ordinal,
        int rootCount, int coefficientCount, int orientation, FixedTriangle triangle, int domainEdgeMask,
        bool hasCompactCell, bool isRational, int denominatorShift)
    {
        HasValue = hasValue; ParameterIsDepth = parameterIsDepth; Ordinal = ordinal; RootCount = rootCount;
        CoefficientCount = coefficientCount; Orientation = orientation; Triangle = triangle;
        DomainEdgeMask = domainEdgeMask;
        HasCompactCell = hasCompactCell; IsRational = isRational; DenominatorShift = denominatorShift;
    }
}

/// <summary>Independently retained finite-domain ray maximum and exact witness.</summary>
internal ref struct FiniteSlabRaySelection
{
    private const int RetainedCellWords = 8;
    internal const int StorageWords = 13 * FiniteSlabPolynomial.Words + RetainedCellWords;
    internal const int SignCount = 13;
    private readonly Span<ulong> _values;
    private readonly Span<sbyte> _signs;
    private bool _parameterIsDepth;
    private int _ordinal, _rootCount, _coefficientCount;
    private FixedTriangle _triangle;
    private int _orientation;
    private int _domainEdgeMask;
    private bool _hasCompactCell, _isRational;
    private int _denominatorShift;
    internal bool HasValue { get; private set; }
    private readonly FiniteSlabPolynomial Polynomial => At(0, 3);
    private readonly FiniteSlabPolynomial Lambda => At(3, 2);
    private readonly FiniteSlabPolynomial Mu => At(5, 2);
    private readonly FiniteSlabPolynomial T => At(7, 2);
    private readonly FiniteSlabPolynomial D => At(9, 2);
    private readonly FiniteSlabPolynomial DepthN => At(11, 1);
    private readonly FiniteSlabPolynomial DepthD => At(12, 1);
    private readonly Span<ulong> RetainedCell => _values[(13 * FiniteSlabPolynomial.Words)..];
    private readonly FiniteSlabPolynomial At(int start, int count) =>
        FiniteSlabPolynomial.At(_values, _signs, start, count);

    internal FiniteSlabRaySelection(Span<ulong> values, Span<sbyte> signs)
    {
        if (values.Length < StorageWords || signs.Length < SignCount)
            throw new ArgumentException("The finite slab ray selection storage is too small.");
        _values = values[..StorageWords]; _signs = signs[..SignCount];
        HasValue = _parameterIsDepth = false; _ordinal = _rootCount = _coefficientCount = _orientation = _domainEdgeMask = 0;
        _triangle = default;
        _hasCompactCell = _isRational = false; _denominatorShift = 0;
    }

    private readonly FiniteAxisValueRoot Restore(Span<ulong> cell)
    {
        if (_hasCompactCell)
        {
            RetainedCell.CopyTo(cell); cell[RetainedCellWords..].Clear();
            return new() { Coefficients = Polynomial.Values[..(_coefficientCount * FiniteSlabPolynomial.Words)],
                Signs = Polynomial.Signs[.._coefficientCount], LowerNumerator = cell,
                DenominatorShift = _denominatorShift, Ordinal = _ordinal, IsRational = _isRational };
        }
        return WideFiniteAxisIntersection.GetKnownFiniteValueRoot(
            Polynomial.Values[..(_coefficientCount * FiniteSlabPolynomial.Words)],
            Polynomial.Signs[.._coefficientCount], _ordinal, _rootCount, cell);
    }
    private readonly int CellWords => Math.Max(RetainedCellWords,
        WideFiniteAxisIntersection.GetFiniteValueRootCellWords(Polynomial.Values, Polynomial.Signs));
    internal readonly FiniteSlabRaySelectionMetadata Metadata => new(HasValue, _parameterIsDepth,
        _ordinal, _rootCount, _coefficientCount, _orientation, _triangle, _domainEdgeMask,
        _hasCompactCell, _isRational, _denominatorShift);
    internal void Restore(FiniteSlabRaySelectionMetadata metadata)
    {
        HasValue = metadata.HasValue; _parameterIsDepth = metadata.ParameterIsDepth; _ordinal = metadata.Ordinal;
        _rootCount = metadata.RootCount; _coefficientCount = metadata.CoefficientCount;
        _orientation = metadata.Orientation; _triangle = metadata.Triangle;
        _domainEdgeMask = metadata.DomainEdgeMask;
        _hasCompactCell = metadata.HasCompactCell; _isRational = metadata.IsRational;
        _denominatorShift = metadata.DenominatorShift;
    }

    /// <summary>
    /// Compares depths sharing the canonical frame. A generic parameter is
    /// u=(1+t)/(2+t); constant-depth families retain a separate rational u.
    /// </summary>
    internal readonly int CompareDepth(scoped in FiniteSlabRaySelection other)
    {
        if (!_parameterIsDepth && !other._parameterIsDepth)
            return Compare(false, default, DepthN, DepthD, false, default, other.DepthN, other.DepthD);
        Span<ulong> firstCell = stackalloc ulong[CellWords];
        Span<ulong> secondCell = stackalloc ulong[other.CellWords];
        FiniteAxisValueRoot first = Restore(firstCell), second = other.Restore(secondCell);
        return Compare(_parameterIsDepth, first, DepthN, DepthD,
            other._parameterIsDepth, second, other.DepthN, other.DepthD);
    }

    private static int Compare(bool firstIsDepth, scoped FiniteAxisValueRoot first,
        scoped FiniteSlabPolynomial firstN, scoped FiniteSlabPolynomial firstD,
        bool secondIsDepth, scoped FiniteAxisValueRoot second,
        scoped FiniteSlabPolynomial secondN, scoped FiniteSlabPolynomial secondD)
    {
        if (firstIsDepth && secondIsDepth)
            return WideFiniteAxisIntersection.CompareFiniteValueRoots(first, second);
        Span<ulong> values = stackalloc ulong[3 * FiniteSlabPolynomial.Words];
        Span<sbyte> signs = stackalloc sbyte[3];
        FiniteSlabPolynomial query = new(values[..(2 * FiniteSlabPolynomial.Words)], signs[..2]);
        if (firstIsDepth || secondIsDepth)
        {
            FiniteSlabPolynomial n = firstIsDepth ? secondN : firstN;
            FiniteSlabPolynomial d = firstIsDepth ? secondD : firstD;
            query.Clear(); query.Add(n, -1); query.Add(d, 1, 1);
            return firstIsDepth ? WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(first, query.Values, query.Signs)
                : -WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(second, query.Values, query.Signs);
        }
        FiniteSlabPolynomial left = query.Slice(0, 1), right = query.Slice(1, 1);
        FiniteSlabPolynomial difference = new(values[(2 * FiniteSlabPolynomial.Words)..], signs[2..]);
        left.Multiply(firstN, secondD); right.Multiply(secondN, firstD);
        difference.Copy(left); difference.Add(right, -1);
        return difference.Signs[0];
    }

    internal void Keep(scoped FiniteAxisValueRoot root, int rootCount,
        scoped FiniteSlabPolynomial lambda, scoped FiniteSlabPolynomial mu,
        scoped FiniteSlabPolynomial t, scoped FiniteSlabPolynomial denominator,
        bool parameterIsDepth, scoped FiniteSlabPolynomial depthN, scoped FiniteSlabPolynomial depthD,
        FixedTriangle triangle, int orientation, int domainEdgeMask)
    {
        if (HasValue)
        {
            Span<ulong> cell = stackalloc ulong[parameterIsDepth || _parameterIsDepth ? CellWords : 0];
            FiniteAxisValueRoot previous = parameterIsDepth || _parameterIsDepth ? Restore(cell) : default;
            // Equal extrema keep the first admitted witness in the explicitly
            // ordered active set and authored triangle traversal.
            if (Compare(parameterIsDepth, root, depthN, depthD,
                    _parameterIsDepth, previous, DepthN, DepthD) <= 0) return;
        }
        Polynomial.Clear(); root.Coefficients.CopyTo(Polynomial.Values); root.Signs.CopyTo(Polynomial.Signs);
        Lambda.Copy(lambda); Mu.Copy(mu); T.Copy(t); D.Copy(denominator);
        DepthN.Copy(depthN); DepthD.Copy(depthD);
        _parameterIsDepth = parameterIsDepth; _ordinal = root.Ordinal; _rootCount = rootCount;
        _coefficientCount = root.Signs.Length; _triangle = triangle; _orientation = orientation; HasValue = true;
        _domainEdgeMask = domainEdgeMask;
        _hasCompactCell = root.DenominatorShift < RetainedCellWords * 64;
        _isRational = root.IsRational; _denominatorShift = root.DenominatorShift;
        RetainedCell.Clear();
        if (_hasCompactCell)
            root.LowerNumerator[..WideArithmetic.GetActiveMagnitudeLength(root.LowerNumerator)].CopyTo(RetainedCell);
    }

    private readonly void BuildAuthoredCoordinate(int axis, FiniteSlabPolynomial numerator, FiniteSlabPolynomial term)
    {
        numerator.Scale(D, Signed320.ExtendValue(Signed192.Raw(_triangle.A[axis])));
        term.Scale(Lambda, Signed320.ExtendValue(WideArithmetic.SubtractSigned192(
            Signed192.Raw(_triangle.B[axis]), Signed192.Raw(_triangle.A[axis])))); numerator.Add(term);
        term.Scale(Mu, Signed320.ExtendValue(WideArithmetic.SubtractSigned192(
            Signed192.Raw(_triangle.C[axis]), Signed192.Raw(_triangle.A[axis])))); numerator.Add(term);
    }

    /// <summary>
    /// Retains the exit in bounded axial/radial components. A stadium endpoint
    /// can exceed the local scalar range even when its translated world anchor
    /// is representable; its exact half-core remainder stays in the anchor.
    /// </summary>
    internal readonly bool TryGetContactAnchor(in FiniteSlabRayFrame frame, out Fixed64 depth, out bool clamped,
        out Vector3d source, out FixedPointAnchor anchor)
    {
        depth = default; source = default; clamped = false; anchor = default;
        if (!HasValue) return false;
        Span<ulong> cell = stackalloc ulong[CellWords]; FiniteAxisValueRoot root = Restore(cell);
        Span<ulong> values = stackalloc ulong[25 * FiniteSlabPolynomial.Words];
        Span<sbyte> signs = stackalloc sbyte[25];
        FiniteSlabPolynomial numerator = FiniteSlabPolynomial.At(values, signs, 0, 5);
        FiniteSlabPolynomial term = FiniteSlabPolynomial.At(values, signs, 5, 5);
        FiniteSlabPolynomial divisor = FiniteSlabPolynomial.At(values, signs, 10, 5);
        FiniteSlabPolynomial square = FiniteSlabPolynomial.At(values, signs, 15, 9);
        FiniteSlabPolynomial scalar = FiniteSlabPolynomial.At(values, signs, 24, 1);
        Signed192 scale = WideArithmetic.AddSigned192(frame.Section.RadialFrame.ShapeFrame.Denominator,
            frame.Section.RadialFrame.ShapeFrame.Denominator);
        divisor.Scale(D, Signed320.ExtendValue(scale));
        WideAxis3 a = frame.Section.Transform(_triangle.A);
        WideAxis3 e = Difference(frame.Section.Transform(_triangle.B), a);
        WideAxis3 f = Difference(frame.Section.Transform(_triangle.C), a);
        Vector3d axial = default, exit = default;
        FixedPointAnchorTerm3d exactTerm = default;
        for (int axis = 0; axis < 3; axis++)
        {
            BuildAuthoredCoordinate(axis, numerator, term);
            if (!TryRoundRatio(ref root, numerator, D, out Fixed64 component)) return false;
            source[axis] = component;
            numerator.Scale(D, Component(a, axis)); term.Scale(Lambda, Component(e, axis)); numerator.Add(term);
            term.Scale(Mu, Component(f, axis)); numerator.Add(term);
            term.Scale(T, Signed(Component(frame.RayNormal, axis), _orientation)); numerator.Add(term);
            bool bounded;
            if (frame.HasCore && axis == 2)
            {
                FiniteSlabPolynomial endpointQuery = square.Slice(0, 5);
                term.Scale(D, frame.CoreCap); endpointQuery.Copy(numerator); endpointQuery.Add(term, -1);
                int denominatorSign = FiniteSlabPolynomial.Sign(ref root, D);
                int endpoint = FiniteSlabPolynomial.Sign(ref root, endpointQuery) * denominatorSign >= 0 ? 1 : 0;
                if (endpoint == 0)
                {
                    endpointQuery.Copy(numerator); endpointQuery.Add(term);
                    if (FiniteSlabPolynomial.Sign(ref root, endpointQuery) * denominatorSign <= 0) endpoint = -1;
                }
                if (endpoint == 0)
                {
                    bounded = TryRoundRatio(ref root, numerator, divisor, out component);
                    System.Diagnostics.Debug.Assert(bounded);
                    axial.Z = component;
                    continue;
                }
                numerator.Add(term, -endpoint);
                axial = TriangleCapsuleSlabEndpointWitnesses.GetCore(Vector2d.Forward,
                    frame.Section.RadialFrame.Length, -endpoint, out exactTerm);
                bounded = TryRoundRatio(ref root, numerator, divisor, out component);
                System.Diagnostics.Debug.Assert(bounded);
                term.Scale(divisor, Signed320.ExtendValue(Signed192.Raw(component)));
                endpointQuery.Copy(numerator); endpointQuery.Add(term, -1);
                if (FiniteSlabPolynomial.Sign(ref root, endpointQuery) == 0)
                {
                    // An integral radial term composes exactly with the odd
                    // half-core remainder retained by the existing anchor.
                    exit.Z = component;
                    continue;
                }
                // Otherwise round the endpoint sum once. Retaining an odd
                // half-core after rounding its radial fraction would round a
                // second time; parity belongs to the complete integer sum.
                exactTerm = default;
                term.Scale(D, Signed(frame.CoreCap, endpoint)); numerator.Add(term);
                term.Scale(divisor, Signed320.ExtendValue(Signed192.Raw(axial.Z))); numerator.Add(term, -1);
                bounded = TryRoundRatio(ref root, numerator, divisor, out component, axial.Z.m_rawValue);
                System.Diagnostics.Debug.Assert(bounded);
                exit.Z = component;
                continue;
            }
            // Exact shape admission bounds local cap/radial/prism components;
            // the authored source above can still exceed its scalar range.
            bounded = TryRoundRatio(ref root, numerator, divisor, out component);
            System.Diagnostics.Debug.Assert(bounded);
            exit[axis] = component;
        }
        anchor = new FixedPointAnchor(frame.Center, frame.SlabRotation, axial, exit, exactTerm);
        if (IsConstant(T) && IsConstant(D))
        {
            Span<ulong> rationalValues = stackalloc ulong[4 * FiniteSlabPolynomial.Words];
            Span<int> rationalSigns = stackalloc int[4];
            ContactQuadratic numeratorValue = ContactQuadratic.At(rationalValues, rationalSigns, 0, FiniteSlabPolynomial.Words);
            ContactQuadratic denominatorValue = ContactQuadratic.At(rationalValues, rationalSigns, 1, FiniteSlabPolynomial.Words);
            numeratorValue.Clear(); denominatorValue.Clear();
            T.Values[..FiniteSlabPolynomial.Words].CopyTo(numeratorValue.Rational);
            D.Values[..FiniteSlabPolynomial.Words].CopyTo(denominatorValue.Rational);
            numeratorValue.Signs[0] = T.Signs[0] * D.Signs[0]; denominatorValue.Signs[0] = 1;
            Span<ulong> metricValue = stackalloc ulong[9]; WideArithmetic.GetMagnitude(frame.RayNormalSquared, metricValue);
            Span<ulong> scaleValue = stackalloc ulong[3];
            WideArithmetic.GetMagnitude(scale, out scaleValue[2], out scaleValue[1], out scaleValue[0]);
            clamped = !ContactQuadratic.TryRoundRootRatio(numeratorValue, denominatorValue, ReadOnlySpan<ulong>.Empty,
                metricValue, scaleValue, out depth);
            if (clamped) depth = Fixed64.MaxValue;
            return true;
        }
        // Depth uses the same conditioned canonical normal as candidate t;
        // numerator²*|N|² / ((2D)²*denominator²) stays exact until this point.
        square.Multiply(T, T);
        Span<ulong> depthValues = stackalloc ulong[27 * FiniteSlabPolynomial.Words];
        Span<sbyte> depthSigns = stackalloc sbyte[27];
        FiniteSlabPolynomial metric = FiniteSlabPolynomial.At(depthValues, depthSigns, 0, 9);
        FiniteSlabPolynomial squaredDivisor = FiniteSlabPolynomial.At(depthValues, depthSigns, 9, 9);
        FiniteSlabPolynomial query = FiniteSlabPolynomial.At(depthValues, depthSigns, 18, 9);
        metric.Scale(square, frame.RayNormalSquared); squaredDivisor.Multiply(divisor, divisor);
        ulong lower = 0, upper = 1UL << 63;
        if (TryGetRatioFloorBounds(ref root, metric, squaredDivisor, 1, 1,
                1UL << 63, true, out ulong minimumDepth, out ulong maximumDepth))
        {
            // This search finds the first integer strictly greater than the
            // exact depth, rather than the floor used by coordinate rounding.
            lower = minimumDepth == 1UL << 63 ? minimumDepth : minimumDepth + 1;
            upper = maximumDepth == 1UL << 63 ? maximumDepth : maximumDepth + 1;
        }
        while (lower < upper)
        {
            ulong mid = lower + ((upper - lower) >> 1);
            if (CompareDepth(ref root, metric, squaredDivisor, query, scalar, mid << 1) >= 0) lower = mid + 1;
            else upper = mid;
        }
        System.Diagnostics.Debug.Assert(lower > 0);
        ulong floor = lower - 1;
        int comparison = CompareDepth(ref root, metric, squaredDivisor, query, scalar, (floor << 1) | 1);
        ulong rounded = floor + (comparison > 0 || comparison == 0 && (floor & 1) != 0 ? 1UL : 0);
        clamped = rounded > long.MaxValue;
        depth = clamped ? Fixed64.MaxValue : Fixed64.FromRaw((long)rounded);
        return true;
    }

    private static int CompareDepth(scoped ref FiniteAxisValueRoot root,
        scoped FiniteSlabPolynomial metric, scoped FiniteSlabPolynomial divisor, scoped FiniteSlabPolynomial query,
        scoped FiniteSlabPolynomial scalar, ulong twiceRaw)
    {
        Span<ulong> factor = stackalloc ulong[FiniteSlabPolynomial.Words]; factor.Clear();
        Fixed64.Multiply64To128(twiceRaw, twiceRaw, out factor[1], out factor[0]);
        scalar.Clear(); factor.CopyTo(scalar.Values); scalar.Signs[0] = twiceRaw == 0 ? (sbyte)0 : (sbyte)1;
        query.Multiply(divisor, scalar);
        query.Add(metric, -4);
        return -FiniteSlabPolynomial.Sign(ref root, query);
    }

    private static bool TryRoundRatio(scoped ref FiniteAxisValueRoot root,
        scoped FiniteSlabPolynomial numerator, scoped FiniteSlabPolynomial denominator, out Fixed64 value,
        long parityOffset = 0)
    {
        value = default;
        if (IsConstant(numerator) && IsConstant(denominator))
        {
            System.Diagnostics.Debug.Assert(denominator.Signs[0] != 0);
            return Fixed64.TryGetSignedRawRatio(numerator.Values[..FiniteSlabPolynomial.Words],
                denominator.Values[..FiniteSlabPolynomial.Words],
                numerator.Signs[0] * denominator.Signs[0] < 0, out value, parityOffset);
        }
        Span<ulong> values = stackalloc ulong[11 * FiniteSlabPolynomial.Words];
        Span<sbyte> signs = stackalloc sbyte[11];
        FiniteSlabPolynomial query = FiniteSlabPolynomial.At(values, signs, 0, 5);
        FiniteSlabPolynomial term = FiniteSlabPolynomial.At(values, signs, 5, 5);
        FiniteSlabPolynomial scalar = FiniteSlabPolynomial.At(values, signs, 10, 1);
        int sign = FiniteSlabPolynomial.Sign(ref root, numerator), denominatorSign = FiniteSlabPolynomial.Sign(ref root, denominator);
        System.Diagnostics.Debug.Assert(denominatorSign != 0);
        if (sign == 0) return true;
        sign *= denominatorSign;
        ulong limit = sign < 0 ? 1UL << 63 : long.MaxValue;
        ulong lower = 0, upper = limit;
        if (TryGetRatioFloorBounds(ref root, numerator, denominator, sign * denominatorSign,
                denominatorSign, limit, false, out ulong minimum, out ulong maximum))
        { lower = minimum; upper = maximum; }
        while (lower < upper)
        {
            ulong mid = lower + ((upper - lower) >> 1) + 1;
            scalar.Clear(); scalar.Values[0] = mid; scalar.Signs[0] = (sbyte)denominatorSign;
            term.Multiply(denominator, scalar); query.Clear(); query.Add(numerator, sign * denominatorSign); query.Add(term, -1);
            if (FiniteSlabPolynomial.Sign(ref root, query) >= 0) lower = mid; else upper = mid - 1;
        }
        scalar.Clear(); scalar.Values[0] = lower; scalar.Signs[0] = (sbyte)denominatorSign;
        term.Multiply(denominator, scalar); query.Clear(); query.Add(numerator, sign * denominatorSign * 2); query.Add(term, -2);
        query.Add(denominator, -denominatorSign);
        int halfway = FiniteSlabPolynomial.Sign(ref root, query);
        ulong rounded = lower + (halfway > 0 || halfway == 0
            && ((lower ^ unchecked((ulong)parityOffset)) & 1) != 0 ? 1UL : 0);
        if (rounded > limit) return false;
        value = Fixed64.FromRaw(sign < 0 ? unchecked(-(long)rounded) : (long)rounded); return true;
    }

    /// <summary>
    /// Bounds the final nonnegative ratio using the existing certified Horner
    /// intervals. These bounds only narrow the exact nearest-even search;
    /// cancellation or a vanishing denominator retains its full-range path.
    /// </summary>
    private static bool TryGetRatioFloorBounds(scoped ref FiniteAxisValueRoot root,
        scoped FiniteSlabPolynomial numerator, scoped FiniteSlabPolynomial denominator,
        int numeratorSign, int denominatorSign, ulong limit, bool squareRoot,
        out ulong minimum, out ulong maximum)
    {
        minimum = 0; maximum = limit;
        const int precision = 59;
        // At most nine coefficients: 59 + 2*ceil(log2(9)) <= 67.
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 70);
        Span<sbyte> signs = stackalloc sbyte[numerator.Count + denominator.Count];
        int numeratorBits = 0, denominatorBits = 0;
        for (int index = 0; index < numerator.Count; index++)
        {
            signs[index] = (sbyte)(numerator.Signs[index] * numeratorSign);
            numeratorBits = Math.Max(numeratorBits, WideArithmetic.GetMagnitudeBitLength(
                numerator.Values.Slice(index * FiniteSlabPolynomial.Words, FiniteSlabPolynomial.Words)));
        }
        for (int index = 0; index < denominator.Count; index++)
        {
            signs[numerator.Count + index] = (sbyte)(denominator.Signs[index] * denominatorSign);
            denominatorBits = Math.Max(denominatorBits, WideArithmetic.GetMagnitudeBitLength(
                denominator.Values.Slice(index * FiniteSlabPolynomial.Words, FiniteSlabPolynomial.Words)));
        }
        WideFiniteAxisIntersection.GetFiniteValueRootNonnegativeBounds(root, numerator.Values,
            signs[..numerator.Count], precision, numeratorBits, out ulong nMin, out ulong nMax);
        WideFiniteAxisIntersection.GetFiniteValueRootNonnegativeBounds(root, denominator.Values,
            signs[numerator.Count..], precision, denominatorBits, out ulong dMin, out ulong dMax);
        if (dMin == 0) return false;
        int shift = numeratorBits - denominatorBits;
        int words = (Math.Abs(shift) + 127) / 64;
        Span<ulong> bank = stackalloc ulong[2 * words];
        Span<ulong> n = bank[..words], d = bank[words..];
        Span<ulong> endpoints = stackalloc ulong[4] { nMin, nMax, dMin, dMax };
        for (int bound = 0; bound < 2; bound++)
        {
            n.Clear(); d.Clear(); int ns = 0, ds = 0;
            WideArithmetic.AddShiftedSignedMagnitude(endpoints.Slice(bound, 1), 1, Math.Max(0, shift), n, ref ns);
            WideArithmetic.AddShiftedSignedMagnitude(endpoints.Slice(3 - bound, 1), 1, Math.Max(0, -shift), d, ref ds);
            ulong value;
            if (squareRoot) value = WideArithmetic.GetRatioFloorSquareRoot(n, d, limit);
            else
            {
                // Negative raw ratios admit the full unsigned 2^63 magnitude.
                // Expand nearest-even endpoint rounding by one integer so it
                // remains a conservative bound on the exact ratio's floor.
                value = Fixed64.TryGetSignedRawRatio(n, d, true, out Fixed64 rounded)
                    ? unchecked((ulong)(-rounded.m_rawValue)) : limit;
                value = Math.Min(value, limit);
                if (bound == 0) value = value == 0 ? 0 : value - 1;
                else if (value < limit) value++;
            }
            if (bound == 0) minimum = value; else maximum = value;
        }
        return true;
    }

    private static bool IsConstant(FiniteSlabPolynomial value)
    {
        for (int index = 1; index < value.Count; index++) if (value.Signs[index] != 0) return false;
        return true;
    }

    private static Signed320 Component(WideAxis3 p, int axis) => axis == 0 ? p.X : axis == 1 ? p.Y : p.Z;
    private static Signed320 Signed(Signed320 value, int sign) => sign < 0 ? WideArithmetic.Negate(value) : value;
    private static WideAxis3 Difference(WideAxis3 p, WideAxis3 q) => new(
        WideArithmetic.SubtractSigned320(p.X, q.X), WideArithmetic.SubtractSigned320(p.Y, q.Y),
        WideArithmetic.SubtractSigned320(p.Z, q.Z));
}
