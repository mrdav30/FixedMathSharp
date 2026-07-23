//=======================================================================
// WideSlabProjection.Entry.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

internal static partial class WideSlabProjection
{
    internal static bool TryGetCapsuleSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        out Vector2d support)
    {
        Signed192 directionLength = GetPlanarDirectionLength(direction);
        if (TryAdmitCapsuleSupport(center, axis, halfLength, radius, slab, direction, directionLength, out support))
            return true;

        bool found = false;
        WidePlanarCandidate best = default;
        AddSphereEndpoint(center, axis, halfLength, radius, -1, slab, direction, directionLength, ref found, ref best);
        AddSphereEndpoint(center, axis, halfLength, radius, 1, slab, direction, directionLength, ref found, ref best);
        AddSpherePlaneCandidate(center, axis, halfLength, radius, -1, slab.Min, direction, directionLength, ref found, ref best);
        AddSpherePlaneCandidate(center, axis, halfLength, radius, 1, slab.Min, direction, directionLength, ref found, ref best);
        AddCapsuleSideCandidate(center, axis, halfLength, radius, slab.Min, direction, ref found, ref best);
        if (slab.Max != slab.Min)
        {
            AddSpherePlaneCandidate(center, axis, halfLength, radius, -1, slab.Max, direction, directionLength, ref found, ref best);
            AddSpherePlaneCandidate(center, axis, halfLength, radius, 1, slab.Max, direction, directionLength, ref found, ref best);
            AddCapsuleSideCandidate(center, axis, halfLength, radius, slab.Max, direction, ref found, ref best);
        }

        return TryCreateResult(found, best, out support);
    }

    internal static bool TryGetCylinderSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        out Vector2d support)
    {
        Signed192 axisLengthSquared = GetAxisLengthSquared(axis);
        if (TryAdmitCylinderSupport(center, axis, halfLength, radius, slab, direction, axisLengthSquared, out support))
            return true;

        bool found = false;
        WidePlanarCandidate best = default;
        AddDiskEndpoint(center, axis, halfLength, radius, -1, slab, direction, axisLengthSquared, ref found, ref best);
        AddDiskEndpoint(center, axis, halfLength, radius, 1, slab, direction, axisLengthSquared, ref found, ref best);
        AddDiskPlaneCandidates(center, axis, halfLength, radius, -1, slab.Min, direction, axisLengthSquared, ref found, ref best);
        AddDiskPlaneCandidates(center, axis, halfLength, radius, 1, slab.Min, direction, axisLengthSquared, ref found, ref best);
        AddCapsuleSideCandidate(center, axis, halfLength, radius, slab.Min, direction, ref found, ref best);
        if (slab.Max != slab.Min)
        {
            AddDiskPlaneCandidates(center, axis, halfLength, radius, -1, slab.Max, direction, axisLengthSquared, ref found, ref best);
            AddDiskPlaneCandidates(center, axis, halfLength, radius, 1, slab.Max, direction, axisLengthSquared, ref found, ref best);
            AddCapsuleSideCandidate(center, axis, halfLength, radius, slab.Max, direction, ref found, ref best);
        }

        return TryCreateResult(found, best, out support);
    }

    internal static bool TryGetConeSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        out Vector2d support)
    {
        // A vertical cone slice is a disk whose radius changes linearly with Y.
        // The general rotated-cone branch is added below after the shared cap
        // candidates so the same exact endpoint ownership is retained.
        if (axis.X == Fixed64.Zero && axis.Z == Fixed64.Zero)
            return TryGetVerticalConeSupport(center, axis, height, radius, slab, direction, out support);

        Signed192 axisLengthSquared = GetAxisLengthSquared(axis);
        if (TryAdmitConeSupport(center, axis, height, radius, slab, direction, axisLengthSquared, out support))
            return true;

        return TryGetRotatedConeSupport(center, axis, height, radius, slab, direction, axisLengthSquared, out support);
    }

    private static bool TryAdmitCapsuleSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        Signed192 directionLength,
        out Vector2d support)
    {
        Signed192 dot = GetPlanarDot(axis, direction);
        bool found = false;
        WidePlanarCandidate best = default;
        if (dot.Sign <= 0)
            AddSphereEndpoint(center, axis, halfLength, radius, -1, slab, direction, directionLength, ref found, ref best);
        if (dot.Sign >= 0)
            AddSphereEndpoint(center, axis, halfLength, radius, 1, slab, direction, directionLength, ref found, ref best);
        return TryCreateResult(found, best, out support);
    }

    private static bool TryAdmitCylinderSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 halfLength,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        Signed192 axisLengthSquared,
        out Vector2d support)
    {
        if (!HasUniqueDiskSupport(axis, direction, axisLengthSquared))
        {
            support = default;
            return false;
        }

        Signed192 dot = GetPlanarDot(axis, direction);
        bool found = false;
        WidePlanarCandidate best = default;
        if (dot.Sign <= 0)
            AddDiskEndpoint(center, axis, halfLength, radius, -1, slab, direction, axisLengthSquared, ref found, ref best);
        if (dot.Sign >= 0)
            AddDiskEndpoint(center, axis, halfLength, radius, 1, slab, direction, axisLengthSquared, ref found, ref best);
        return TryCreateResult(found, best, out support);
    }

    private static bool TryAdmitConeSupport(
        Vector3d center,
        Vector3d axis,
        Fixed64 height,
        Fixed64 radius,
        FixedRange slab,
        Vector2d direction,
        Signed192 axisLengthSquared,
        out Vector2d support)
    {
        if (!HasUniqueDiskSupport(axis, direction, axisLengthSquared))
        {
            support = default;
            return false;
        }

        bool apexFound = false;
        WidePlanarCandidate apex = default;
        AddConeApex(center, axis, height, slab, direction, ref apexFound, ref apex);
        bool baseFound = false;
        WidePlanarCandidate baseCandidate = default;
        AddConeBaseDisk(center, axis, height, radius, slab, direction, axisLengthSquared, ref baseFound, ref baseCandidate);
        if (!apexFound || !baseFound)
        {
            support = default;
            return false;
        }

        bool found = true;
        KeepBest(baseCandidate, direction, ref found, ref apex);
        return TryCreateResult(true, apex, out support);
    }

    private static bool HasUniqueDiskSupport(Vector3d axis, Vector2d direction, Signed192 axisLengthSquared)
    {
        Signed192 dot = GetPlanarDot(axis, direction);
        Signed320 gx = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(axisLengthSquared, Raw(direction.X)),
            WideArithmetic.MultiplySigned192(Raw(axis.X), dot));
        Signed320 gy = WideArithmetic.SubtractSigned320(
            default,
            WideArithmetic.MultiplySigned192(Raw(axis.Y), dot));
        Signed320 gz = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(axisLengthSquared, Raw(direction.Y)),
            WideArithmetic.MultiplySigned192(Raw(axis.Z), dot));
        return !SumSquares(gx, gy, gz).IsZero;
    }
}
