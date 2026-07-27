//=======================================================================
// WideOrientedBox.CylinderCapFace.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contact generation for oriented box vs. centered cylinder cap-face collisions.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct LocalCapContact : IEquatable<LocalCapContact>
    {
        internal LocalCapContact(Signed320 u, Signed320 v)
        {
            U = u;
            V = v;
        }

        internal Signed320 U { get; }
        internal Signed320 V { get; }

        public bool Equals(LocalCapContact other) =>
            U.Equals(other.U) && V.Equals(other.V);
    }

    #endregion

    // Rounded canonical axes can differ by a few raw units from the exact
    // quaternion basis even when authored from the same rotation.
    private static readonly Fixed64 ParallelFaceAlignmentThreshold =
        Fixed64.One - Fixed64.FromRaw(64L);

    internal static void GetCenteredCylinderCapFaceContacts(
        Vector3d boxCenter,
        FixedQuaternion boxOrientation,
        Vector3d boxHalfExtents,
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        FixedContactAnchors primary,
        CenteredCylinderContactFeature feature,
        Span<FixedContactLocalPoints> contacts,
        out int count)
    {
        count = 0;
        if (!feature.IsCapFace)
            return;

        int faceAxis = feature.BoxFaceAxis;

        // A normalized axis scaled by half of a representable positive length
        // is always representable when the radial extent is zero.
        FixedPointAnchor capCenterAnchor =
            WideGeometry.GetCenteredCylinderSupportAnchor(
            cylinderCenter,
            cylinderRotation,
            localCylinderAxis,
            cylinderLength,
            Fixed64.Zero,
            -primary.Normal,
            feature.CylinderCapSign);
        _ = capCenterAnchor.TryGetOffsetFrom(
            new FixedPointAnchor(
                cylinderCenter,
                FixedQuaternion.Identity,
                Vector3d.Zero),
            out Vector3d capCenterOffset);

        RationalBasis basis = new(boxOrientation);
        GetPointProjections(
            cylinderCenter,
            boxCenter,
            basis,
            out Signed320 centerX,
            out Signed320 centerY,
            out Signed320 centerZ);
        GetDirectionProjections(
            capCenterOffset,
            basis,
            out Signed320 capX,
            out Signed320 capY,
            out Signed320 capZ);
        centerX = WideArithmetic.AddSigned320(centerX, capX);
        centerY = WideArithmetic.AddSigned320(centerY, capY);
        centerZ = WideArithmetic.AddSigned320(centerZ, capZ);

        Signed320 extentX = GetExtentNumerator(
            boxHalfExtents.X,
            basis.Denominator);
        Signed320 extentY = GetExtentNumerator(
            boxHalfExtents.Y,
            basis.Denominator);
        Signed320 extentZ = GetExtentNumerator(
            boxHalfExtents.Z,
            basis.Denominator);
        Signed320 radius = WideArithmetic.MultiplySigned192(
            Signed192.Raw(cylinderRadius),
            basis.Denominator);
        GetFaceCoordinates(
            faceAxis,
            centerX,
            centerY,
            centerZ,
            extentX,
            extentY,
            extentZ,
            out Signed320 centerU,
            out Signed320 centerV,
            out Signed320 extentU,
            out Signed320 extentV);
        Span<LocalCapContact> localContacts =
            stackalloc LocalCapContact[4];
        KeepCapExtreme(
            centerU,
            centerV,
            extentU,
            extentV,
            radius,
            positive: false,
            primaryAxisIsU: true,
            localContacts,
            ref count);
        KeepCapExtreme(
            centerU,
            centerV,
            extentU,
            extentV,
            radius,
            positive: true,
            primaryAxisIsU: true,
            localContacts,
            ref count);
        KeepCapExtreme(
            centerV,
            centerU,
            extentV,
            extentU,
            radius,
            positive: false,
            primaryAxisIsU: false,
            localContacts,
            ref count);
        KeepCapExtreme(
            centerV,
            centerU,
            extentV,
            extentU,
            radius,
            positive: true,
            primaryAxisIsU: false,
            localContacts,
            ref count);

        FixedPointAnchor cylinderSupport =
            WideGeometry.GetCenteredCylinderSupportAnchor(
                cylinderCenter,
                cylinderRotation,
                localCylinderAxis,
                cylinderLength,
                Fixed64.Zero,
                -primary.Normal,
                feature.CylinderCapSign);
        Fixed64 supportAxial =
            Vector3d.Dot(cylinderSupport.LocalPoint, localCylinderAxis);
        int outputCount = 0;
        for (int index = 0; index < count; index++)
        {
            LocalCapContact local = localContacts[index];
            GetLocalCoordinates(
                faceAxis,
                local.U,
                local.V,
                GetSignedFaceExtent(
                    faceAxis,
                    feature.BoxFaceSign,
                    extentX,
                    extentY,
                    extentZ),
                out Signed320 localX,
                out Signed320 localY,
                out Signed320 localZ);
            // Clipping constrains this point to the box face and the
            // overlapping cylinder cap, so both local frames are bounded by
            // their representable authored extents.
            Vector3d boxLocalPoint = GetRationalLocalPoint(
                localX,
                localY,
                localZ,
                basis.Denominator);

            var boxPoint = new FixedPointAnchor(
                boxCenter,
                boxOrientation,
                boxLocalPoint);
            _ = boxPoint.TryGetLocalPointIn(
                cylinderCenter,
                cylinderRotation,
                out Vector3d cylinderLocalPoint);

            Fixed64 currentAxial =
                Vector3d.Dot(cylinderLocalPoint, localCylinderAxis);
            cylinderLocalPoint +=
                localCylinderAxis * (supportAxial - currentAxial);
            contacts[outputCount++] = new FixedContactLocalPoints(
                boxLocalPoint,
                cylinderLocalPoint);
        }

        count = outputCount;
    }

    private static Vector3d GetRationalLocalPoint(
        Signed320 xNumerator,
        Signed320 yNumerator,
        Signed320 zNumerator,
        Signed192 denominator)
    {
        Signed576 denominatorWide = Signed576.ExtendValue(
            Signed320.ExtendValue(denominator));
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(xNumerator),
            denominatorWide,
            out Fixed64 x);
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(yNumerator),
            denominatorWide,
            out Fixed64 y);
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(zNumerator),
            denominatorWide,
            out Fixed64 z);
        return new Vector3d(x, y, z);
    }

    private static void GetFaceCoordinates(
        int faceAxis,
        Signed320 x,
        Signed320 y,
        Signed320 z,
        Signed320 extentX,
        Signed320 extentY,
        Signed320 extentZ,
        out Signed320 u,
        out Signed320 v,
        out Signed320 extentU,
        out Signed320 extentV)
    {
        if (faceAxis == 0)
        {
            u = y;
            v = z;
            extentU = extentY;
            extentV = extentZ;
            return;
        }
        if (faceAxis == 1)
        {
            u = x;
            v = z;
            extentU = extentX;
            extentV = extentZ;
            return;
        }

        u = x;
        v = y;
        extentU = extentX;
        extentV = extentY;
    }

    private static void KeepCapExtreme(
        Signed320 centerPrimary,
        Signed320 centerSecondary,
        Signed320 extentPrimary,
        Signed320 extentSecondary,
        Signed320 radius,
        bool positive,
        bool primaryAxisIsU,
        Span<LocalCapContact> contacts,
        ref int count)
    {
        Signed320 secondary = ClampToExtent(
            centerSecondary,
            extentSecondary);
        Signed320 secondaryDelta = WideArithmetic.SubtractSigned320(
            secondary,
            centerSecondary);
        Signed576 radialSquared = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(radius, radius),
            WideArithmetic.MultiplySigned320(
                secondaryDelta,
                secondaryDelta));
        Signed576 remaining = WideArithmetic.GetFloorSquareRoot(
            Signed704.ExtendValue(radialSquared));
        Signed576 center = Signed576.ExtendValue(centerPrimary);
        Signed576 extreme = positive
            ? WideArithmetic.AddSigned576(center, remaining)
            : WideArithmetic.SubtractSigned576(center, remaining);
        Signed576 minimum = Signed576.ExtendValue(
            WideArithmetic.SubtractSigned320(default, extentPrimary));
        Signed576 maximum = Signed576.ExtendValue(extentPrimary);
        if (CompareSigned(extreme, minimum) < 0)
            extreme = minimum;
        else if (CompareSigned(extreme, maximum) > 0)
            extreme = maximum;
        // The extreme was just clamped between Signed320 extents.
        Signed320 primary = new(
            extreme.Word4,
            extreme.Word3,
            extreme.Word2,
            extreme.Word1,
            extreme.Word0);

        LocalCapContact candidate = primaryAxisIsU
            ? new LocalCapContact(primary, secondary)
            : new LocalCapContact(secondary, primary);
        for (int index = 0; index < count; index++)
        {
            if (contacts[index].Equals(candidate))
                return;
        }

        contacts[count++] = candidate;
    }

    private static Signed320 GetSignedFaceExtent(
        int faceAxis,
        int faceSign,
        Signed320 extentX,
        Signed320 extentY,
        Signed320 extentZ)
    {
        Signed320 extent = faceAxis == 0
            ? extentX
            : faceAxis == 1
                ? extentY
                : extentZ;
        return faceSign < 0
            ? WideArithmetic.SubtractSigned320(default, extent)
            : extent;
    }

    private static void GetLocalCoordinates(
        int faceAxis,
        Signed320 u,
        Signed320 v,
        Signed320 face,
        out Signed320 x,
        out Signed320 y,
        out Signed320 z)
    {
        if (faceAxis == 0)
        {
            x = face;
            y = u;
            z = v;
            return;
        }
        if (faceAxis == 1)
        {
            x = u;
            y = face;
            z = v;
            return;
        }

        x = u;
        y = v;
        z = face;
    }

    private static Signed320 ClampToExtent(
        Signed320 value,
        Signed320 extent)
    {
        Signed320 minimum =
            WideArithmetic.SubtractSigned320(default, extent);
        if (CompareSigned(value, minimum) < 0)
            return minimum;
        return CompareSigned(value, extent) > 0 ? extent : value;
    }
}
