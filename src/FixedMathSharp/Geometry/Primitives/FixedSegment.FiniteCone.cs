//=======================================================================
// FixedSegment.FiniteCone.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Segment-vs-finite-cone intersection tests.
/// </content>
public partial struct FixedSegment
{
    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// finite cone authored from its apex toward the center of its flat base.
    /// </summary>
    /// <param name="apex">The cone apex.</param>
    /// <param name="apexToBaseDirection">The normalized direction from apex to base.</param>
    /// <param name="height">
    /// The positive parametric axis extent; the conceptual base center is
    /// <c>apex + apexToBaseDirection * height</c>.
    /// </param>
    /// <param name="baseRadius">The nonnegative base radius.</param>
    /// <param name="entryParameter">The first intersecting segment parameter in [0, 1].</param>
    /// <param name="exitParameter">The last intersecting segment parameter in [0, 1].</param>
    /// <exception cref="ArgumentException">
    /// <paramref name="apexToBaseDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    public readonly bool TryGetFiniteConeIntersectionInterval(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetFiniteConeIntersectionInterval(
            apex,
            apexToBaseDirection,
            height,
            baseRadius,
            out entryParameter,
            out exitParameter,
            out _,
            out _);

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// finite apex-authored cone and reports exact endpoint containment.
    /// </summary>
    /// <remarks>
    /// <paramref name="startContained"/> includes the side, apex, and flat base.
    /// <paramref name="endContainedStrict"/> excludes every boundary. The
    /// classifications and quadratic solve remain wide until the final
    /// deterministic parameter conversion.
    /// </remarks>
    public readonly bool TryGetFiniteConeIntersectionInterval(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));
        return WideFiniteConeIntersection.TryGetApexInterval(
            this,
            apex,
            apexToBaseDirection,
            height,
            baseRadius,
            out entryParameter,
            out exitParameter,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// centered finite cone.
    /// </summary>
    /// <param name="center">The midpoint between the apex and flat-base center.</param>
    /// <param name="baseToApexDirection">The normalized direction from base to apex.</param>
    /// <param name="height">
    /// The positive full parametric axis extent between base and apex.
    /// </param>
    /// <param name="baseRadius">The nonnegative base radius.</param>
    /// <param name="entryParameter">The first intersecting segment parameter in [0, 1].</param>
    /// <param name="exitParameter">The last intersecting segment parameter in [0, 1].</param>
    /// <exception cref="ArgumentException">
    /// <paramref name="baseToApexDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    public readonly bool TryGetCenteredFiniteConeIntersectionInterval(
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCenteredFiniteConeIntersectionInterval(
            center,
            baseToApexDirection,
            height,
            baseRadius,
            out entryParameter,
            out exitParameter,
            out _,
            out _);

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// centered finite cone and reports exact endpoint containment.
    /// </summary>
    public readonly bool TryGetCenteredFiniteConeIntersectionInterval(
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateFiniteCone(
            baseToApexDirection,
            height,
            baseRadius,
            nameof(baseToApexDirection),
            nameof(height));
        return WideFiniteConeIntersection.TryGetCenteredInterval(
            this,
            center,
            baseToApexDirection,
            height,
            baseRadius,
            out entryParameter,
            out exitParameter,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// finite cone authored from its apex toward its flat base.
    /// </summary>
    /// <remarks>
    /// Distances are solved directly from the wide polynomial; they are not
    /// reconstructed from rounded Q32.32 parameters.
    /// </remarks>
    public readonly bool TryGetFiniteConeIntersectionDistanceInterval(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetFiniteConeIntersectionDistanceInterval(
            apex,
            apexToBaseDirection,
            height,
            baseRadius,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects an
    /// apex-authored finite cone and reports exact endpoint containment.
    /// </summary>
    public readonly bool TryGetFiniteConeIntersectionDistanceInterval(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));
        return WideFiniteConeIntersection.TryGetApexDistanceInterval(
            this,
            apex,
            apexToBaseDirection,
            height,
            baseRadius,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered finite cone.
    /// </summary>
    public readonly bool TryGetCenteredFiniteConeIntersectionDistanceInterval(
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetCenteredFiniteConeIntersectionDistanceInterval(
            center,
            baseToApexDirection,
            height,
            baseRadius,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered finite cone and reports exact endpoint containment.
    /// </summary>
    public readonly bool TryGetCenteredFiniteConeIntersectionDistanceInterval(
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        ValidateFiniteCone(
            baseToApexDirection,
            height,
            baseRadius,
            nameof(baseToApexDirection),
            nameof(height));
        return WideFiniteConeIntersection.TryGetCenteredDistanceInterval(
            this,
            center,
            baseToApexDirection,
            height,
            baseRadius,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds deterministic lattice witnesses for the first and last points where
    /// this segment intersects an apex-authored finite cone.
    /// </summary>
    /// <remarks>
    /// The solver uses the largest positive Q32.32 interval scale before exact
    /// authored-chord interpolation, so long segments do not amplify ordinary
    /// Q32.32 parameter rounding into coarse spatial error.
    /// </remarks>
    public readonly bool TryGetFiniteConeIntersectionPointInterval(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Vector3d entryPoint,
        out Vector3d exitPoint)
    {
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));
        Fixed64 intervalScale = Start == End ? Fixed64.Zero : Fixed64.MaxValue;
        if (!WideFiniteConeIntersection.TryGetApexDistanceInterval(
                this,
                apex,
                apexToBaseDirection,
                height,
                baseRadius,
                intervalScale,
                out Fixed64 entry,
                out Fixed64 exit,
                out _,
                out _))
        {
            entryPoint = default;
            exitPoint = default;
            return false;
        }

        entryPoint = GetPointAtDistance(entry, intervalScale);
        exitPoint = GetPointAtDistance(exit, intervalScale);
        return true;
    }

    /// <summary>
    /// Finds the intersecting lattice point with the smallest axial distance
    /// from an apex-authored cone's apex.
    /// </summary>
    /// <remarks>
    /// The exact segment-axis projection selects the appropriate end of the
    /// admitted interval before one high-resolution authored-chord interpolation.
    /// </remarks>
    public readonly bool TryGetFiniteConeIntersectionMinimumAxialPoint(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Vector3d point)
    {
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));
        Fixed64 intervalScale = Start == End ? Fixed64.Zero : Fixed64.MaxValue;
        if (!WideFiniteConeIntersection.TryGetApexDistanceInterval(
                this,
                apex,
                apexToBaseDirection,
                height,
                baseRadius,
                intervalScale,
                out Fixed64 entry,
                out Fixed64 exit,
                out _,
                out _))
        {
            point = default;
            return false;
        }

        Signed192 axialVelocity = WideGeometry.GetDifferenceDotProduct3D(
            End.X, Start.X, End.Y, Start.Y, End.Z, Start.Z,
            apexToBaseDirection.X, Fixed64.Zero,
            apexToBaseDirection.Y, Fixed64.Zero,
            apexToBaseDirection.Z, Fixed64.Zero);
        point = GetPointAtDistance(axialVelocity.Sign < 0 ? exit : entry, intervalScale);
        return true;
    }

    /// <summary>
    /// Finds deterministic lattice witnesses for the first and last points where
    /// this segment intersects a centered finite cone.
    /// </summary>
    public readonly bool TryGetCenteredFiniteConeIntersectionPointInterval(
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Vector3d entryPoint,
        out Vector3d exitPoint)
    {
        ValidateFiniteCone(
            baseToApexDirection,
            height,
            baseRadius,
            nameof(baseToApexDirection),
            nameof(height));
        Fixed64 intervalScale = Start == End ? Fixed64.Zero : Fixed64.MaxValue;
        if (!WideFiniteConeIntersection.TryGetCenteredDistanceInterval(
                this,
                center,
                baseToApexDirection,
                height,
                baseRadius,
                intervalScale,
                out Fixed64 entry,
                out Fixed64 exit,
                out _,
                out _))
        {
            entryPoint = default;
            exitPoint = default;
            return false;
        }

        entryPoint = GetPointAtDistance(entry, intervalScale);
        exitPoint = GetPointAtDistance(exit, intervalScale);
        return true;
    }

    /// <summary>
    /// Returns whether a point lies strictly inside or inclusively within a
    /// finite cone authored from its apex toward its flat base.
    /// </summary>
    public static bool ContainsPointInFiniteCone(
        Vector3d point,
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        bool strict = false)
    {
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));
        return WideFiniteConeIntersection.ContainsPointInApexCone(
            point,
            apex,
            apexToBaseDirection,
            height,
            baseRadius,
            strict);
    }

    /// <summary>
    /// Returns whether a point lies strictly inside or inclusively within a
    /// centered finite cone whose direction points from base to apex.
    /// </summary>
    public static bool ContainsPointInCenteredFiniteCone(
        Vector3d point,
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        bool strict = false)
    {
        ValidateFiniteCone(
            baseToApexDirection,
            height,
            baseRadius,
            nameof(baseToApexDirection),
            nameof(height));
        return WideFiniteConeIntersection.ContainsPointInCenteredCone(
            point,
            center,
            baseToApexDirection,
            height,
            baseRadius,
            strict);
    }

    private static void ValidateFiniteCone(
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        string axisParameterName,
        string heightParameterName)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cone axis direction must be normalized.", axisParameterName);
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(heightParameterName);
        if (baseRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(baseRadius));
    }
}
