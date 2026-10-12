//=======================================================================
// CapsulePlaneRaySelection.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Metadata over independently retained winner coefficient/sign banks.</summary>
internal readonly struct CapsulePlaneRaySelectionMetadata
{
    internal readonly bool HasValue, UsesValueRoot;
    internal readonly int Primitive;
    internal readonly CapsulePlaneRayEventSource Source;
    internal readonly CapsulePlaneRayEvent Event;
    internal readonly CapsulePlaneRayValueMetadata Value;
    internal CapsulePlaneRaySelectionMetadata(bool hasValue, bool usesValueRoot, int primitive,
        CapsulePlaneRayEventSource source, CapsulePlaneRayEvent descriptor, CapsulePlaneRayValueMetadata value)
    { HasValue = hasValue; UsesValueRoot = usesValueRoot; Primitive = primitive; Source = source; Event = descriptor; Value = value; }
}

/// <summary>Borrowed exact point and depth storage for an admitted capsule plane ray.</summary>
internal ref struct CapsulePlaneRaySelection
{
    internal const int FieldWords = 64;
    internal const int RootWords = 48;
    private const int QuadraticStorageWords = 12 * FieldWords + RootWords;
    internal const int StorageWords = QuadraticStorageWords + CapsulePlaneRayValue.StorageWords;
    internal const int SignCount = 12;
    internal const int ValueSignCount = CapsulePlaneRayValue.SignCount;
    internal readonly Span<ulong> Values;
    internal readonly Span<int> Signs;
    internal readonly Span<sbyte> ValueSigns;
    internal bool HasValue;
    private bool usesValueRoot;
    private CapsulePlaneRayValueMetadata valueMetadata;
    internal int Primitive;
    internal CapsulePlaneRayEventSource CurrentSource, MaximumSource;
    internal CapsulePlaneRayEvent CurrentEvent, MaximumEvent;
    internal readonly ContactQuadratic X => ContactQuadratic.At(Values, Signs, 0, FieldWords);
    internal readonly ContactQuadratic Y => ContactQuadratic.At(Values, Signs, 1, FieldWords);
    internal readonly ContactQuadratic Z => ContactQuadratic.At(Values, Signs, 2, FieldWords);
    internal readonly ContactQuadratic Denominator => ContactQuadratic.At(Values, Signs, 3, FieldWords);
    internal readonly ContactQuadratic DepthNumerator => ContactQuadratic.At(Values, Signs, 4, FieldWords);
    internal readonly ContactQuadratic DepthDenominator => ContactQuadratic.At(Values, Signs, 5, FieldWords);
    internal readonly ReadOnlySpan<ulong> Root => Values.Slice(12 * FieldWords, RootWords);
    internal readonly CapsulePlaneRaySelectionMetadata Metadata => new(HasValue, usesValueRoot, Primitive,
        MaximumSource, MaximumEvent, valueMetadata);

    /// <summary>Compares exact base anchors without materializing or evaluating their exits.</summary>
    internal readonly int ComparePoint(scoped in CapsulePlaneRaySelection other, scoped in CapsulePlaneRayFrame frame) =>
        frame.ComparePoints(X, Y, Z, Denominator, Root, other.X, other.Y, other.Z, other.Denominator, other.Root);

    internal CapsulePlaneRaySelection(Span<ulong> values, Span<int> signs, Span<sbyte> valueSigns)
    {
        if (values.Length < StorageWords || signs.Length < SignCount || valueSigns.Length < ValueSignCount)
            throw new ArgumentException("The capsule ray selection storage is too small.");
        Values = values[..StorageWords]; Signs = signs[..SignCount]; HasValue = false; Primitive = 0;
        ValueSigns = valueSigns[..ValueSignCount]; usesValueRoot = false; valueMetadata = default;
        CurrentSource = MaximumSource = default; CurrentEvent = MaximumEvent = default;
    }

    private readonly CapsulePlaneRayValue Value
    {
        get
        {
            var value = new CapsulePlaneRayValue(Values[QuadraticStorageWords..], ValueSigns);
            value.Restore(valueMetadata); return value;
        }
    }

    internal void Keep(scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator,
        scoped ContactQuadratic numerator, scoped ContactQuadratic divisor, scoped ReadOnlySpan<ulong> root, int primitive,
        in CapsulePlaneRayFrame frame)
    {
        if (HasValue)
        {
            int comparison = usesValueRoot ? -Value.CompareQuadratic(numerator, divisor, root)
                : ContactQuadratic.CompareRatios(numerator, divisor, root, DepthNumerator, DepthDenominator, Root);
            if (comparison < 0) return;
            if (comparison == 0)
            {
                int order = frame.ComparePoints(x, y, z, denominator, root, X, Y, Z, Denominator, Root);
                if (order > 0 || order == 0 && primitive >= Primitive) return;
            }
        }
        x.CopyTo(X); y.CopyTo(Y); z.CopyTo(Z); denominator.CopyTo(Denominator);
        numerator.CopyTo(DepthNumerator); divisor.CopyTo(DepthDenominator);
        Span<ulong> destination = Values.Slice(12 * FieldWords, RootWords);
        root.CopyTo(destination); destination[root.Length..].Clear();
        HasValue = true; Primitive = primitive;
        usesValueRoot = false; valueMetadata = default;
        MaximumSource = CurrentSource; MaximumEvent = CurrentEvent;
    }

    internal readonly int CompareDepth(scoped in CapsulePlaneRaySelection other)
    {
        if (usesValueRoot)
            return other.usesValueRoot ? Value.Compare(other.Value)
                : Value.CompareQuadratic(other.DepthNumerator, other.DepthDenominator, other.Root);
        return other.usesValueRoot ? -other.Value.CompareQuadratic(DepthNumerator, DepthDenominator, Root)
            : ContactQuadratic.CompareRatios(DepthNumerator, DepthDenominator, Root,
                other.DepthNumerator, other.DepthDenominator, other.Root);
    }

    internal void KeepEvaluated(scoped CapsulePlaneRaySelection other, in CapsulePlaneRayFrame frame)
    {
        if (!other.HasValue) return;
        CurrentSource = other.MaximumSource; CurrentEvent = other.MaximumEvent;
        if (other.usesValueRoot)
        {
            CapsulePlaneRayValue value = other.Value;
            KeepValue(other.X, other.Y, other.Z, other.Denominator, other.Root, value, other.Primitive, frame);
            return;
        }
        Keep(other.X, other.Y, other.Z, other.Denominator, other.DepthNumerator,
            other.DepthDenominator, other.Root, other.Primitive, frame);
    }

    internal void Restore(CapsulePlaneRaySelectionMetadata metadata)
    {
        HasValue = metadata.HasValue; usesValueRoot = metadata.UsesValueRoot; Primitive = metadata.Primitive;
        CurrentSource = MaximumSource = metadata.Source; CurrentEvent = MaximumEvent = metadata.Event;
        valueMetadata = metadata.Value;
    }

    internal void KeepValue(scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z,
        scoped ContactQuadratic denominator, scoped ReadOnlySpan<ulong> root,
        scoped CapsulePlaneRayValue value, int primitive, in CapsulePlaneRayFrame frame)
    {
        if (HasValue)
        {
            int comparison = usesValueRoot ? value.Compare(Value)
                : value.CompareQuadratic(DepthNumerator, DepthDenominator, Root);
            if (comparison < 0) return;
            if (comparison == 0)
            {
                int order = frame.ComparePoints(x, y, z, denominator, root, X, Y, Z, Denominator, Root);
                if (order > 0 || order == 0 && primitive >= Primitive) return;
            }
        }
        x.CopyTo(X); y.CopyTo(Y); z.CopyTo(Z); denominator.CopyTo(Denominator);
        Span<ulong> destination = Values.Slice(12 * FieldWords, RootWords);
        root.CopyTo(destination); destination[root.Length..].Clear();
        var retained = new CapsulePlaneRayValue(Values[QuadraticStorageWords..], ValueSigns);
        retained.CopyFrom(value); valueMetadata = retained.Metadata;
        HasValue = usesValueRoot = true; Primitive = primitive;
        MaximumSource = CurrentSource; MaximumEvent = CurrentEvent;
    }

    internal bool TryGetDepth(in CapsulePlaneRayFrame frame, out Fixed64 depth)
        => TryGetDepth(frame, out depth, out _);

    internal bool TryGetDepth(in CapsulePlaneRayFrame frame, out Fixed64 depth, out bool clamped)
    {
        depth = default; clamped = false;
        if (!HasValue) return false;
        Signed192 doubleScale = WideArithmetic.AddSigned192(frame.Finite.ShapeFrame.Denominator, frame.Finite.ShapeFrame.Denominator);
        if (usesValueRoot) return Value.TryGetDepth(frame.NormalSquared, doubleScale, out depth, out clamped);
        Span<ulong> metric = stackalloc ulong[9]; WideArithmetic.GetMagnitude(frame.NormalSquared, metric);
        Span<ulong> scale = stackalloc ulong[3]; WideArithmetic.GetMagnitude(doubleScale, out scale[2], out scale[1], out scale[0]);
        clamped = !ContactQuadratic.TryRoundRootRatio(DepthNumerator, DepthDenominator, Root, metric, scale, out depth);
        if (clamped) depth = Fixed64.MaxValue;
        return true;
    }

    internal bool ContainsTrianglePoint(in CapsulePlaneRayFrame frame, FixedTriangle triangle) =>
        HasValue && frame.ContainsTrianglePoint(triangle, X, Y, Z, Denominator, Root);

    internal bool TryGetAuthoredPoint(in CapsulePlaneRayFrame frame, Vector3d meshOrigin, FixedQuaternion meshRotation,
        Vector3d center, FixedQuaternion rotation, out Vector3d point)
    {
        point = default;
        if (!HasValue) return false;
        Span<ulong> values = stackalloc ulong[2 * FieldWords]; Span<int> signs = stackalloc int[2];
        ContactQuadratic d = ContactQuadratic.At(values, signs, 0, FieldWords);
        ContactQuadratic.Scale(Denominator, GetCoordinateScale(frame), d);
        return QuadraticPointMaterialization.TryGetPointInRigidFrame(center, rotation, meshOrigin, meshRotation,
            X, Y, Z, d, Root, out point);
    }

    internal bool TryGetExitAnchor(in CapsulePlaneRayFrame frame, int orientation,
        Vector3d center, FixedQuaternion rotation, out FixedPointAnchor anchor,
        out Fixed64 depth, out bool clamped)
    {
        anchor = default; depth = default; clamped = false;
        if (orientation is not (1 or -1)) throw new ArgumentOutOfRangeException(nameof(orientation));
        if (!HasValue) return false;
        bool depthRepresented = TryGetDepth(frame, out depth, out clamped);
        System.Diagnostics.Debug.Assert(depthRepresented);
        Span<ulong> values = stackalloc ulong[16 * FieldWords]; Span<int> signs = stackalloc int[16];
        ContactQuadratic x = ContactQuadratic.At(values, signs, 0, FieldWords), y = ContactQuadratic.At(values, signs, 1, FieldWords);
        ContactQuadratic z = ContactQuadratic.At(values, signs, 2, FieldWords), d = ContactQuadratic.At(values, signs, 3, FieldWords);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 4, FieldWords), work = ContactQuadratic.At(values, signs, 5, FieldWords);
        ContactQuadratic zero = ContactQuadratic.At(values, signs, 6, FieldWords), scaled = ContactQuadratic.At(values, signs, 7, FieldWords);
        zero.Clear();
        Signed192 coordinateScale = WideArithmetic.AddSigned192(frame.Finite.ShapeFrame.Denominator, frame.Finite.ShapeFrame.Denominator);
        WideAxis3 direction = orientation > 0 ? frame.Normal : new(WideArithmetic.Negate(frame.Normal.X),
            WideArithmetic.Negate(frame.Normal.Y), WideArithmetic.Negate(frame.Normal.Z));
        int endpoint = frame.Finite.Length == Fixed64.Zero ? 0 : Primitive == 1 ? -1 : Primitive == 2 ? 1 : 0;
        Vector3d axial = default, radial;
        if (usesValueRoot)
        {
            CapsulePlaneRayValue value = Value;
            // A body primitive exists only for a positive-length core;
            // zero-core producers admit the sphere as the upper cap.
            if (Primitive == 0)
            {
                ContactQuadratic.Scale(Denominator, Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap), work);
                Y.CopyTo(term); term.Add(work, -1);
                ContactQuadratic.Scale(Denominator, Signed576.ExtendValue(direction.Y), scaled);
                if (value.SignLinear(term, scaled, Root) == 0) endpoint = 1;
                else { Y.CopyTo(term); term.Add(work); if (value.SignLinear(term, scaled, Root) == 0) endpoint = -1; }
            }
            if (endpoint == 0 && frame.Finite.Length != Fixed64.Zero)
            {
                bool axialRepresented = value.TryGetPoint(zero, Y, zero, Denominator, Root,
                    new WideAxis3(default, direction.Y, default), coordinateScale, out axial);
                System.Diagnostics.Debug.Assert(axialRepresented);
                y.Clear();
            }
            else
            {
                Y.CopyTo(y);
                Signed320 cap = endpoint < 0 ? WideArithmetic.Negate(frame.Finite.ShapeFrame.Cap)
                    : endpoint > 0 ? frame.Finite.ShapeFrame.Cap : default;
                ContactQuadratic.Scale(Denominator, Signed576.ExtendValue(cap), term); y.Add(term, -1);
            }
            WideAxis3 radialDirection = endpoint == 0 && frame.Finite.Length != Fixed64.Zero
                ? new(direction.X, default, direction.Z) : direction;
            bool radialRepresented = value.TryGetPoint(X, y, Z, Denominator, Root, radialDirection, coordinateScale, out radial);
            System.Diagnostics.Debug.Assert(radialRepresented);
        }
        else
        {
            SetExitCoordinates(frame, orientation, x, y, z, d, term, work);
            ContactQuadratic.Scale(d, Signed576.ExtendValue(frame.Finite.ShapeFrame.Cap), work);
            if (Primitive == 0)
            {
                y.CopyTo(term); term.Add(work, -1);
                if (term.Sign(Root) == 0) endpoint = 1;
                else { y.CopyTo(term); term.Add(work); if (term.Sign(Root) == 0) endpoint = -1; }
            }
            ContactQuadratic.Scale(d, GetCoordinateScale(frame), scaled);
            if (endpoint == 0 && frame.Finite.Length != Fixed64.Zero)
            {
                bool axialRepresented = QuadraticPointMaterialization.TryGetWorldPoint(Vector3d.Zero, FixedQuaternion.Identity,
                    zero, y, zero, scaled, Root, out axial);
                System.Diagnostics.Debug.Assert(axialRepresented);
                y.Clear();
            }
            else y.Add(work, -endpoint);
            bool radialRepresented = QuadraticPointMaterialization.TryGetWorldPoint(Vector3d.Zero, FixedQuaternion.Identity,
                x, y, z, scaled, Root, out radial);
            System.Diagnostics.Debug.Assert(radialRepresented);
        }
        FixedPointAnchorTerm3d exactTerm = default;
        if (endpoint != 0)
        {
            Fixed64 signedLength = endpoint > 0 ? frame.Finite.Length : -frame.Finite.Length;
            axial = new Vector3d(Fixed64.Zero, signedLength / Fixed64.Two, Fixed64.Zero);
            // The endpoint's half-raw remainder stays separate from the
            // independently rounded, bounded radial exit.
            exactTerm = FixedPointAnchorTerm3d.CreateCenteredAxisSupport(Vector3d.Up, signedLength,
                Vector3d.Zero, Fixed64.Zero, axial, Vector3d.Zero);
        }
        anchor = new FixedPointAnchor(center, rotation, axial, radial, exactTerm);
        return true;
    }

    private void SetExitCoordinates(in CapsulePlaneRayFrame frame, int orientation, scoped ContactQuadratic x,
        scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic d,
        scoped ContactQuadratic term, scoped ContactQuadratic work)
    {
        ContactQuadratic.Multiply(Denominator, DepthDenominator, Root, d);
        ContactQuadratic.Multiply(Denominator, DepthNumerator, Root, term);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic source = axis == 0 ? X : axis == 1 ? Y : Z;
            ContactQuadratic target = axis == 0 ? x : axis == 1 ? y : z;
            ContactQuadratic.Multiply(source, DepthDenominator, Root, target);
            Signed320 direction = axis == 0 ? frame.Normal.X : axis == 1 ? frame.Normal.Y : frame.Normal.Z;
            ContactQuadratic.Scale(term, Signed576.ExtendValue(direction), work); target.Add(work, orientation);
        }
    }

    private static Signed576 GetCoordinateScale(in CapsulePlaneRayFrame frame) =>
        Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.AddSigned192(
            frame.Finite.ShapeFrame.Denominator, frame.Finite.ShapeFrame.Denominator)));
}
