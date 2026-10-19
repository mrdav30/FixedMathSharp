//=======================================================================
// CapsulePlaneRayValue.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <summary>Copied metadata for a borrowed positive capsule boundary-ray value.</summary>
internal readonly struct CapsulePlaneRayValueMetadata
{
    internal readonly int ValueShift, Ordinal, RootCount, PolynomialCount, DenominatorShift;
    internal readonly bool IsRational, HasCompactCell, HasValue;

    internal CapsulePlaneRayValueMetadata(int valueShift, int ordinal, int rootCount, int polynomialCount,
        int denominatorShift, bool rational, bool compact)
    {
        ValueShift = valueShift; Ordinal = ordinal; RootCount = rootCount; PolynomialCount = polynomialCount;
        DenominatorShift = denominatorShift; IsRational = rational; HasCompactCell = compact; HasValue = true;
    }
}

/// <summary>
/// One positive outward ray root of a quadratic over an independent quadratic
/// point field. The norm polynomial and selected ordinal borrow caller storage.
/// </summary>
internal ref struct CapsulePlaneRayValue
{
    // Opposite rays through edge-stationary quadratic witnesses have the largest
    // norm: coefficients <3190 bits, normalization adds <=402, and the point
    // radicand is <2256. Its complete degree-four norm stays <9600 bits.
    internal const int PolynomialWords = 160, SignCount = 5, CellWords = 8;
    internal const int StorageWords = SignCount * PolynomialWords + CellWords;
    internal static int GetStorageWords(int polynomialWords) => SignCount * polynomialWords + CellWords;
    private readonly int _words;
    private readonly Span<ulong> _polynomial, _cell;
    private readonly Span<sbyte> _signs;
    private CapsulePlaneRayValueMetadata _metadata;
    internal CapsulePlaneRayValueMetadata Metadata => _metadata;
    internal bool HasValue => _metadata.HasValue;

    internal CapsulePlaneRayValue(Span<ulong> storage, Span<sbyte> signs, int polynomialWords = PolynomialWords)
    {
        _words = polynomialWords;
        _polynomial = storage[..(SignCount * polynomialWords)];
        _cell = storage.Slice(SignCount * polynomialWords, CellWords);
        _signs = signs[..SignCount]; _metadata = default;
    }

    internal void Restore(CapsulePlaneRayValueMetadata metadata) => _metadata = metadata;

    internal void CopyFrom(scoped CapsulePlaneRayValue other)
    {
        _polynomial.Clear();
        for (int index = 0; index < SignCount; index++)
        {
            ReadOnlySpan<ulong> coefficient = other._polynomial.Slice(index * other._words, other._words);
            coefficient[..WideArithmetic.GetActiveMagnitudeLength(coefficient)].CopyTo(_polynomial.Slice(index * _words, _words));
        }
        other._signs.CopyTo(_signs); other._cell.CopyTo(_cell);
        _metadata = other._metadata;
    }

    /// <summary>Retains the positive root of a*t²+2*b*t+c with nonnegative outward derivative.</summary>
    internal bool TrySetUpperRoot(scoped ContactQuadratic a, scoped ContactQuadratic b, scoped ContactQuadratic c,
        scoped ReadOnlySpan<ulong> pointRoot, int valueShift)
    {
        _metadata = default;
        Span<ulong> rational = stackalloc ulong[3 * _words];
        Span<ulong> radical = stackalloc ulong[3 * _words];
        Span<sbyte> rationalSigns = stackalloc sbyte[3], radicalSigns = stackalloc sbyte[3];
        rational.Clear(); radical.Clear(); rationalSigns.Clear(); radicalSigns.Clear();
        SetCoefficient(c, rational, rationalSigns, radical, radicalSigns, 0, 0);
        SetCoefficient(b, rational, rationalSigns, radical, radicalSigns, 1, valueShift + 1);
        SetCoefficient(a, rational, rationalSigns, radical, radicalSigns, 2, 2 * valueShift);
        bool rationalRay = ((byte)radicalSigns[0] | (byte)radicalSigns[1] | (byte)radicalSigns[2]) == 0
            || WideArithmetic.GetActiveMagnitudeLength(pointRoot) == 0;
        if (rationalRay)
        {
            // Rational ray coefficients need only their original quadratic.
            // Squaring it would duplicate every root and double its height.
            _polynomial.Clear(); _signs.Clear();
            rational.CopyTo(_polynomial); rationalSigns.CopyTo(_signs);
        }
        else
            BuildNorm(rational, rationalSigns, radical, radicalSigns, pointRoot, _polynomial, _signs);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(_polynomial, _signs);
        Span<ulong> batchCells = stackalloc ulong[64]; Span<int> shifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(_polynomial, _signs, batchCells, shifts);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(_polynomial, _signs)];
        Span<ulong> derivativeRational = stackalloc ulong[2 * _words];
        Span<ulong> derivativeRadical = stackalloc ulong[2 * _words];
        Span<sbyte> derivativeRationalSigns = stackalloc sbyte[2], derivativeRadicalSigns = stackalloc sbyte[2];
        WideFiniteAxisIntersection.DifferentiateFiniteAxisPolynomial(rational, rationalSigns, derivativeRational, derivativeRationalSigns);
        WideFiniteAxisIntersection.DifferentiateFiniteAxisPolynomial(radical, radicalSigns, derivativeRadical, derivativeRadicalSigns);
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, _polynomial, _signs, cell);
            // The norm also contains the conjugate field's roots. Unsquared
            // admission and the outward derivative identify the intended exit.
            if (!rationalRay && SignAtRoot(ref root, rational, rationalSigns, radical, radicalSigns, pointRoot, knownNormRoot: true) != 0
                || SignAtRoot(ref root, derivativeRational, derivativeRationalSigns,
                    derivativeRadical, derivativeRadicalSigns, pointRoot) < 0)
                continue;
            bool compact = root.DenominatorShift < CellWords * 64;
            _cell.Clear();
            if (compact) root.LowerNumerator[..WideArithmetic.GetActiveMagnitudeLength(root.LowerNumerator)].CopyTo(_cell);
            _metadata = new CapsulePlaneRayValueMetadata(valueShift, ordinal, roots.Count, root.Signs.Length,
                root.DenominatorShift, root.IsRational, compact);
            return true;
        }
        return false;
    }

    internal int Compare(scoped CapsulePlaneRayValue other)
    {
        Span<ulong> firstCell = stackalloc ulong[CellCapacity()];
        Span<ulong> secondCell = stackalloc ulong[other.CellCapacity()];
        FiniteAxisValueRoot first = GetRoot(_polynomial, _signs, _cell, _metadata, firstCell),
            second = GetRoot(other._polynomial, other._signs, other._cell, other._metadata, secondCell);
        int shift = Math.Max(_metadata.ValueShift, other._metadata.ValueShift);
        Span<ulong> firstPolynomial = stackalloc ulong[SignCount * _words];
        Span<ulong> secondPolynomial = stackalloc ulong[SignCount * other._words];
        first.Coefficients.CopyTo(firstPolynomial); second.Coefficients.CopyTo(secondPolynomial);
        int firstScale = shift - _metadata.ValueShift, secondScale = shift - other._metadata.ValueShift;
        first.Coefficients = firstPolynomial[..first.Coefficients.Length];
        second.Coefficients = secondPolynomial[..second.Coefficients.Length];
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(firstPolynomial[..first.Coefficients.Length], first.Signs.Length, firstScale);
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(secondPolynomial[..second.Coefficients.Length], second.Signs.Length, secondScale);
        first.DenominatorShift += firstScale; second.DenominatorShift += secondScale;
        return WideFiniteAxisIntersection.CompareFiniteValueRoots(first, second);
    }

    internal int Compare(scoped CapsulePlaneRayValue other, Signed320 divisor, Signed320 otherDivisor)
    {
        System.Diagnostics.Debug.Assert(divisor.Sign > 0 && otherDivisor.Sign > 0);
        Span<ulong> firstCell = stackalloc ulong[CellCapacity()];
        Span<ulong> secondCell = stackalloc ulong[other.CellCapacity()];
        FiniteAxisValueRoot first = GetRoot(_polynomial, _signs, _cell, _metadata, firstCell),
            second = GetRoot(other._polynomial, other._signs, other._cell, other._metadata, secondCell);
        int shift = Math.Max(_metadata.ValueShift, other._metadata.ValueShift);
        int firstScale = shift - _metadata.ValueShift, secondScale = shift - other._metadata.ValueShift;
        Span<ulong> firstDenominator = stackalloc ulong[5 + (firstScale + 63) / 64];
        Span<ulong> secondDenominator = stackalloc ulong[5 + (secondScale + 63) / 64];
        Span<ulong> magnitude = stackalloc ulong[5];
        firstDenominator.Clear(); secondDenominator.Clear();
        WideArithmetic.GetMagnitude(divisor, out magnitude[4], out magnitude[3], out magnitude[2], out magnitude[1], out magnitude[0]);
        int sign = 0;
        WideArithmetic.AddShiftedSignedMagnitude(magnitude, 1, firstScale, firstDenominator, ref sign);
        WideArithmetic.GetMagnitude(otherDivisor, out magnitude[4], out magnitude[3], out magnitude[2], out magnitude[1], out magnitude[0]);
        sign = 0;
        WideArithmetic.AddShiftedSignedMagnitude(magnitude, 1, secondScale, secondDenominator, ref sign);
        // t = alpha*2^ValueShift. A common binary factor cancels, leaving
        // original roots divided by D*2^(commonShift-ValueShift).
        return WideFiniteAxisIntersection.CompareFiniteValueRoots(first, firstDenominator, second, secondDenominator);
    }

    /// <summary>Compares this positive value with an exact quadratic ratio whose denominator is positive.</summary>
    internal int CompareQuadratic(scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        scoped ReadOnlySpan<ulong> root)
    {
        Span<ulong> values = stackalloc ulong[2 * numerator.FieldWords]; Span<int> signs = stackalloc int[2];
        ContactQuadratic constant = new(values, signs); numerator.CopyTo(constant, -1);
        return SignLinear(constant, denominator, root);
    }

    /// <summary>Exact sign of constant+slope*t, including an independent quadratic coefficient field.</summary>
    internal int SignLinear(scoped ContactQuadratic constant, scoped ContactQuadratic slope,
        scoped ReadOnlySpan<ulong> root)
    {
        Span<ulong> cell = stackalloc ulong[CellCapacity()];
        FiniteAxisValueRoot parameter = GetRoot(_polynomial, _signs, _cell, _metadata, cell);
        return LinearSign(ref parameter, constant, slope, root, _metadata.ValueShift);
    }

    private static int LinearSign(scoped ref FiniteAxisValueRoot parameter, scoped ContactQuadratic constant,
        scoped ContactQuadratic slope, scoped ReadOnlySpan<ulong> root, int valueShift)
    {
        int _words = Math.Max(parameter.Coefficients.Length / parameter.Signs.Length, 2 * Math.Max(constant.FieldWords, slope.FieldWords) + (root.Length + 1));
        Span<ulong> rational = stackalloc ulong[2 * _words], radical = stackalloc ulong[2 * _words];
        Span<sbyte> rationalSigns = stackalloc sbyte[2], radicalSigns = stackalloc sbyte[2];
        rational.Clear(); radical.Clear(); rationalSigns.Clear(); radicalSigns.Clear();
        SetCoefficient(constant, rational, rationalSigns, radical, radicalSigns, 0, 0);
        SetCoefficient(slope, rational, rationalSigns, radical, radicalSigns, 1, valueShift);
        return SignAtRoot(ref parameter, rational, rationalSigns, radical, radicalSigns, root);
    }

    /// <summary>Rounds capsule-local (q+t*direction)/coordinateScale once at the final coordinate lattice.</summary>
    internal bool TryGetPoint(scoped ContactQuadratic qx, scoped ContactQuadratic qy,
        scoped ContactQuadratic qz, scoped ContactQuadratic qd, scoped ReadOnlySpan<ulong> pointRoot,
        WideDirection3d direction, Signed320 coordinateScale, out Vector3d point)
    {
        Span<ulong> values = stackalloc ulong[8 * _words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic denominator = ContactQuadratic.At(values, signs, 0, _words);
        ContactQuadratic sx = ContactQuadratic.At(values, signs, 1, _words);
        ContactQuadratic sy = ContactQuadratic.At(values, signs, 2, _words);
        ContactQuadratic sz = ContactQuadratic.At(values, signs, 3, _words);
        ContactQuadratic.Scale(qd, Signed576.ExtendValue(coordinateScale), denominator);
        ContactQuadratic.Scale(qd, direction.X, sx);
        ContactQuadratic.Scale(qd, direction.Y, sy);
        ContactQuadratic.Scale(qd, direction.Z, sz);
        Span<ulong> cell = stackalloc ulong[CellCapacity()];
        FiniteAxisValueRoot parameter = GetRoot(_polynomial, _signs, _cell, _metadata, cell);
        if (!TryRoundCoordinate(ref parameter, qx, sx, denominator, pointRoot, _metadata.ValueShift, out Fixed64 px)
            || !TryRoundCoordinate(ref parameter, qy, sy, denominator, pointRoot, _metadata.ValueShift, out Fixed64 py)
            || !TryRoundCoordinate(ref parameter, qz, sz, denominator, pointRoot, _metadata.ValueShift, out Fixed64 pz))
        {
            point = default;
            return false;
        }
        point = new Vector3d(px, py, pz);
        return true;
    }

    private static bool TryRoundCoordinate(scoped ref FiniteAxisValueRoot parameter, scoped ContactQuadratic constant,
        scoped ContactQuadratic slope, scoped ContactQuadratic denominator, scoped ReadOnlySpan<ulong> root,
        int valueShift, out Fixed64 coordinate)
    {
        coordinate = default;
        int _words = Math.Max(constant.FieldWords, Math.Max(slope.FieldWords, denominator.FieldWords)) + 2;
        Span<ulong> values = stackalloc ulong[6 * _words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic query = ContactQuadratic.At(values, signs, 0, _words);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 1, _words);
        ContactQuadratic doubledSlope = ContactQuadratic.At(values, signs, 2, _words);
        slope.CopyTo(doubledSlope); doubledSlope.Add(slope);
        Signed192 upper = WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(Signed192.Signed(long.MaxValue),
            Signed192.Signed(long.MaxValue)), Signed192.Signed(1));
        Signed192 lower = WideArithmetic.SubtractSigned192(WideArithmetic.AddSigned192(Signed192.Signed(long.MinValue),
            Signed192.Signed(long.MinValue)), Signed192.Signed(1));
        if (CompareCoordinate(ref parameter, constant, doubledSlope, denominator, root, upper, query, term, valueShift) >= 0
            || CompareCoordinate(ref parameter, constant, doubledSlope, denominator, root, lower, query, term, valueShift) < 0)
            return false;
        // A zero direction retains the original point field; its coordinate
        // needs no value-root search after the exact range checks above.
        if ((slope.Signs[0] | slope.Signs[1]) == 0)
        {
            coordinate = ContactQuadratic.RoundRatio(constant, denominator, root);
            return true;
        }
        long lowerFloor = long.MinValue, upperFloor = long.MaxValue;
        if (parameter.DenominatorShift < CellWords * 64)
            GetCoordinateFloorBounds(parameter, constant, slope, denominator, root, valueShift,
                out lowerFloor, out upperFloor);
        ulong low = unchecked((ulong)lowerFloor) ^ (1UL << 63);
        ulong high = unchecked((ulong)upperFloor) ^ (1UL << 63);
        while (low < high)
        {
            ulong middle = low + ((high - low) >> 1) + 1;
            long raw = unchecked((long)(middle ^ (1UL << 63)));
            Signed192 threshold = WideArithmetic.AddSigned192(Signed192.Signed(raw), Signed192.Signed(raw));
            if (CompareCoordinate(ref parameter, constant, doubledSlope, denominator, root, threshold, query, term, valueShift) >= 0) low = middle;
            else high = middle - 1;
        }
        long floor = unchecked((long)(low ^ (1UL << 63)));
        Signed192 half = WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(Signed192.Signed(floor),
            Signed192.Signed(floor)), Signed192.Signed(1));
        int comparison = CompareCoordinate(ref parameter, constant, doubledSlope, denominator, root, half, query, term, valueShift);
        coordinate = Fixed64.FromRaw(floor + (comparison > 0 || comparison == 0 && (floor & 1) != 0 ? 1L : 0L));
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)] // Return interval scratch before exact root signs.
    private static void GetCoordinateFloorBounds(scoped FiniteAxisValueRoot parameter,
        scoped ContactQuadratic constant, scoped ContactQuadratic slope, scoped ContactQuadratic denominator,
        scoped ReadOnlySpan<ulong> root, int valueShift, out long lower, out long upper)
    {
        // t=2^s*alpha and alpha is in [L,L+1]/2^h. With positive D the
        // affine quotient's extrema are its endpoints, even for negative slope.
        // Keep both point-field values exact; their independent CQ enclosures
        // only narrow the search. Cancellation and range uncertainty fall back.
        int commonShift = Math.Max(0, parameter.DenominatorShift - valueShift);
        int endpointShift = Math.Max(0, valueShift - parameter.DenominatorShift);
        int endpointWords = WideArithmetic.GetActiveMagnitudeLength(parameter.LowerNumerator)
            + 1 + (endpointShift + 63) / 64;
        int commonWords = commonShift / 64 + 1;
        int words = Math.Max(Math.Max(ContactQuadratic.ActiveWords(constant), ContactQuadratic.ActiveWords(denominator))
            + commonWords, ContactQuadratic.ActiveWords(slope) + endpointWords) + 1;
        Span<ulong> values = stackalloc ulong[6 * words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic numerator = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic divisor = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 2, words);
        Span<ulong> common = stackalloc ulong[commonWords], endpoint = stackalloc ulong[endpointWords];
        common.Clear(); common[commonShift / 64] = 1UL << (commonShift & 63);
        ContactQuadratic.Scale(denominator, common, 1, divisor);
        lower = long.MaxValue; upper = long.MinValue;
        Span<ulong> one = stackalloc ulong[1] { 1 };
        for (int index = 0; index < 2; index++)
        {
            endpoint.Clear(); int sign = 0;
            WideArithmetic.AddShiftedSignedMagnitude(parameter.LowerNumerator, 1, endpointShift, endpoint, ref sign);
            if (index != 0 && !parameter.IsRational)
                WideArithmetic.AddShiftedSignedMagnitude(one, 1, endpointShift, endpoint, ref sign);
            ContactQuadratic.Scale(constant, common, 1, numerator);
            ContactQuadratic.Scale(slope, endpoint, 1, term); numerator.Add(term);
            long endpointLower = long.MinValue, endpointUpper = long.MaxValue;
            ContactQuadratic.RefineRoundRatioBounds(numerator, divisor, root, ref endpointLower, ref endpointUpper);
            lower = Math.Min(lower, endpointLower); upper = Math.Max(upper, endpointUpper);
        }
    }

    private static int CompareCoordinate(scoped ref FiniteAxisValueRoot parameter, scoped ContactQuadratic constant,
        scoped ContactQuadratic doubledSlope, scoped ContactQuadratic denominator, scoped ReadOnlySpan<ulong> root,
        Signed192 twiceRaw, scoped ContactQuadratic query, scoped ContactQuadratic term, int valueShift)
    {
        constant.CopyTo(query); query.Add(constant);
        ContactQuadratic.Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(twiceRaw)), term);
        query.Add(term, -1);
        return LinearSign(ref parameter, query, doubledSlope, root, valueShift);
    }

    /// <summary>Rounds t*sqrt(metric)/scale to nonnegative raw depth; scale and metric are positive.</summary>
    internal bool TryGetDepth(Signed832 metric, Signed320 scale, out Fixed64 depth, out bool overflow)
    {
        Span<ulong> cell = stackalloc ulong[CellCapacity()]; FiniteAxisValueRoot parameter = GetRoot(_polynomial, _signs, _cell, _metadata, cell);
        Span<ulong> metricWords = stackalloc ulong[13]; WideArithmetic.GetMagnitude(metric, metricWords);
        Span<ulong> scaleWords = stackalloc ulong[5]; WideArithmetic.GetMagnitude(scale, out scaleWords[4], out scaleWords[3], out scaleWords[2], out scaleWords[1], out scaleWords[0]);
        Span<ulong> squaredScale = stackalloc ulong[10]; WideArithmetic.MultiplyMagnitudes(scaleWords, scaleWords, squaredScale);
        int maximum = CompareDepth(ref parameter, metricWords, squaredScale, unchecked((ulong)long.MaxValue << 1), _metadata.ValueShift);
        if (maximum >= 0) { depth = Fixed64.MaxValue; overflow = maximum > 0; return true; }
        ulong lower = 0, upper = 1UL << 63;
        if (_metadata.HasCompactCell)
        {
            GetDepthFloorBounds(parameter, metricWords, squaredScale, _metadata.ValueShift, out lower, out upper);
            // The search finds the first inadmissible integer, then subtracts one.
            lower++; upper++;
        }
        while (lower < upper)
        {
            ulong middle = lower + ((upper - lower) >> 1);
            if (CompareDepth(ref parameter, metricWords, squaredScale, middle << 1, _metadata.ValueShift) >= 0) lower = middle + 1;
            else upper = middle;
        }
        ulong floor = lower - 1;
        int comparison = CompareDepth(ref parameter, metricWords, squaredScale, (floor << 1) | 1, _metadata.ValueShift);
        depth = Fixed64.FromRaw((long)(floor + (comparison > 0 || comparison == 0 && (floor & 1) != 0 ? 1UL : 0)));
        overflow = false; return true;
    }

    private static void GetDepthFloorBounds(scoped FiniteAxisValueRoot parameter,
        scoped ReadOnlySpan<ulong> metric, scoped ReadOnlySpan<ulong> squaredScale, int valueShift,
        out ulong lower, out ulong upper)
    {
        // The retained dyadic cell encloses t. Squaring its nonnegative
        // endpoints bounds raw depth; exact signs still decide the final half.
        int cellWords = WideArithmetic.GetActiveMagnitudeLength(parameter.LowerNumerator) + 1;
        int shift = 2 * (parameter.DenominatorShift - valueShift);
        int words = Math.Max(2 * cellWords + metric.Length, squaredScale.Length)
            + (Math.Abs(shift) + 63) / 64 + 1;
        Span<ulong> endpoint = stackalloc ulong[cellWords], square = stackalloc ulong[2 * cellWords];
        Span<ulong> product = stackalloc ulong[words], numerator = stackalloc ulong[words], denominator = stackalloc ulong[words];
        endpoint.Clear(); parameter.LowerNumerator[..(cellWords - 1)].CopyTo(endpoint);
        denominator.Clear();
        int sign = 0;
        WideArithmetic.AddShiftedSignedMagnitude(squaredScale, 1, Math.Max(0, shift), denominator, ref sign);
        lower = upper = 0;
        Span<ulong> one = stackalloc ulong[1] { 1 };
        for (int index = 0; index < 2; index++)
        {
            WideArithmetic.MultiplyMagnitudes(endpoint, endpoint, square);
            WideArithmetic.MultiplyMagnitudes(square, metric, product);
            numerator.Clear(); sign = 0;
            WideArithmetic.AddShiftedSignedMagnitude(product, 1, Math.Max(0, -shift), numerator, ref sign);
            ulong floor = WideArithmetic.GetRatioFloorSquareRoot(numerator, denominator, long.MaxValue);
            if (index == 0) lower = floor; else upper = floor;
            WideArithmetic.AddMagnitudeInto(one, endpoint);
        }
    }

    private static int CompareDepth(scoped ref FiniteAxisValueRoot parameter, scoped ReadOnlySpan<ulong> metric,
        scoped ReadOnlySpan<ulong> squaredScale, ulong twiceRaw, int valueShift)
    {
        int _words = Math.Max(PolynomialWords, parameter.Coefficients.Length / parameter.Signs.Length);
        Span<ulong> query = stackalloc ulong[3 * _words]; Span<sbyte> signs = stackalloc sbyte[3];
        query.Clear(); signs.Clear();
        int sign = 0;
        WideArithmetic.AddShiftedSignedMagnitude(metric, 1, 2 * valueShift + 2,
            query.Slice(2 * _words, _words), ref sign); signs[2] = (sbyte)sign;
        Span<ulong> threshold = stackalloc ulong[2]; Fixed64.Multiply64To128(twiceRaw, twiceRaw, out threshold[1], out threshold[0]);
        WideArithmetic.MultiplyMagnitudes(threshold, squaredScale, query[.._words]);
        signs[0] = WideArithmetic.GetActiveMagnitudeLength(query[.._words]) == 0 ? (sbyte)0 : (sbyte)-1;
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref parameter, query, signs);
    }

    private int CellCapacity() => _metadata.HasCompactCell ? CellWords
        : WideFiniteAxisIntersection.GetFiniteValueRootCellWords(_polynomial[..(_metadata.PolynomialCount * _words)], _signs[.._metadata.PolynomialCount]);

    private static FiniteAxisValueRoot GetRoot(ReadOnlySpan<ulong> polynomial, ReadOnlySpan<sbyte> polynomialSigns,
        ReadOnlySpan<ulong> retainedCell, CapsulePlaneRayValueMetadata metadata, Span<ulong> cell)
    {
        int _words = polynomial.Length / polynomialSigns.Length;
        ReadOnlySpan<ulong> coefficients = polynomial[..(metadata.PolynomialCount * _words)];
        ReadOnlySpan<sbyte> signs = polynomialSigns[..metadata.PolynomialCount];
        if (!metadata.HasCompactCell)
            return WideFiniteAxisIntersection.GetKnownFiniteValueRoot(coefficients, signs, metadata.Ordinal, metadata.RootCount, cell);
        retainedCell.CopyTo(cell);
        return new FiniteAxisValueRoot { Coefficients = coefficients, Signs = signs, LowerNumerator = cell,
            DenominatorShift = metadata.DenominatorShift, Ordinal = metadata.Ordinal, IsRational = metadata.IsRational };
    }

    private static void SetCoefficient(scoped ContactQuadratic value, Span<ulong> rational, Span<sbyte> rationalSigns,
        Span<ulong> radical, Span<sbyte> radicalSigns, int index, int shift)
    {
        int _words = rational.Length / rationalSigns.Length;
        int rationalSign = 0, radicalSign = 0;
        WideArithmetic.AddShiftedSignedMagnitude(value.Rational, value.Signs[0], shift,
            rational.Slice(index * _words, _words), ref rationalSign);
        WideArithmetic.AddShiftedSignedMagnitude(value.Radical, value.Signs[1], shift,
            radical.Slice(index * _words, _words), ref radicalSign);
        rationalSigns[index] = (sbyte)rationalSign; radicalSigns[index] = (sbyte)radicalSign;
    }

    private static void BuildNorm(scoped ReadOnlySpan<ulong> rational, scoped ReadOnlySpan<sbyte> rationalSigns,
        scoped ReadOnlySpan<ulong> radical, scoped ReadOnlySpan<sbyte> radicalSigns, scoped ReadOnlySpan<ulong> root,
        Span<ulong> polynomial, Span<sbyte> signs)
    {
        int words = polynomial.Length / signs.Length;
        Span<ulong> product = stackalloc ulong[words], radicalSquared = stackalloc ulong[polynomial.Length];
        Span<sbyte> radicalSquaredSigns = stackalloc sbyte[signs.Length];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(rational, rationalSigns, rational, rationalSigns,
            polynomial, signs, product);
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(radical, radicalSigns, radical, radicalSigns,
            radicalSquared, radicalSquaredSigns, product);
        for (int index = 0; index < signs.Length; index++)
        {
            WideArithmetic.MultiplyMagnitudes(radicalSquared.Slice(index * words, words), root, product);
            int sign = signs[index];
            WideArithmetic.AddShiftedSignedMagnitude(product, -radicalSquaredSigns[index], 0,
                polynomial.Slice(index * words, words), ref sign); signs[index] = (sbyte)sign;
        }
    }

    private static int SignAtRoot(scoped ref FiniteAxisValueRoot parameter,
        scoped ReadOnlySpan<ulong> rational, scoped ReadOnlySpan<sbyte> rationalSigns,
        scoped ReadOnlySpan<ulong> radical, scoped ReadOnlySpan<sbyte> radicalSigns, scoped ReadOnlySpan<ulong> root,
        bool knownNormRoot = false)
    {
        int rationalSign = WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref parameter, rational, rationalSigns);
        int radicalSign = WideArithmetic.GetActiveMagnitudeLength(root) == 0 ? 0
            : WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref parameter, radical, radicalSigns);
        if (rationalSign == 0) return radicalSign;
        if (radicalSign == 0 || rationalSign == radicalSign) return rationalSign;
        // The defining norm already proves equal magnitudes. Opposite signs
        // admit its unsquared branch without proving that same norm zero again.
        if (knownNormRoot) return 0;
        int count = 2 * rationalSigns.Length - 1;
        int inputWords = rational.Length / rationalSigns.Length, active = 0;
        for (int index = 0; index < rationalSigns.Length; index++)
            active = Math.Max(active, Math.Max(WideArithmetic.GetActiveMagnitudeLength(rational.Slice(index * inputWords, inputWords)),
                WideArithmetic.GetActiveMagnitudeLength(radical.Slice(index * inputWords, inputWords))));
        int words = 2 * active + WideArithmetic.GetActiveMagnitudeLength(root) + 1;
        Span<ulong> norm = stackalloc ulong[count * words]; Span<sbyte> signs = stackalloc sbyte[count];
        BuildNorm(rational, rationalSigns, radical, radicalSigns, root, norm, signs);
        return rationalSign * WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref parameter, norm, signs);
    }
}
