using System;
using System.Runtime.CompilerServices;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class FixedConvexHullRelationsTests
{
    [Fact]
    public void FilteredPenetration_ShouldKeepTheOtherSignedExitBeforeDiscardingAnAxis()
    {
        bool rejectedNear = false, admittedFar = false;
        bool Admit(in WidePointSpanPenetration candidate)
        {
            WideAxis3 normal = candidate.Negate ? -candidate.Axis : candidate.Axis;
            if (!normal.Y.IsZero || !normal.Z.IsZero) return false;
            if (normal.X.Sign > 0) { rejectedNear = true; return false; }
            admittedFar = true; return true;
        }
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(Vector3d.Zero, FixedQuaternion.Identity,
            CubePoints, CubeTriangles, CubeEdges, Vector3d.Right * (Fixed64.One + Fixed64.Half), FixedQuaternion.Identity,
            CubePoints, CubeTriangles, CubeEdges, out var penetration, admission: Admit));
        Assert.True(rejectedNear); Assert.True(admittedFar);
        Assert.Equal((Fixed64)3 + Fixed64.Half, penetration.GetRoundedDepth(out bool clamped));
        Assert.False(clamped);
        Assert.True((penetration.Negate ? -penetration.Axis : penetration.Axis).X.Sign < 0);
    }

    [Fact]
    public void FilteredPenetration_ShouldSeparateOverlapTruthFromResponseAdmission()
    {
        static bool Reject(in WidePointSpanPenetration candidate) => false;
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(Vector3d.Zero, FixedQuaternion.Identity,
            CubePoints, CubeTriangles, CubeEdges, Vector3d.Right, FixedQuaternion.Identity,
            CubePoints, CubeTriangles, CubeEdges, out var overlap, admission: Reject));
        Assert.False(overlap.HasValue);
        Assert.False(WideOrientedBox.TryGetConvexHullPenetration(Vector3d.Zero, FixedQuaternion.Identity,
            CubePoints, CubeTriangles, CubeEdges, Vector3d.Right * (Fixed64)3, FixedQuaternion.Identity,
            CubePoints, CubeTriangles, CubeEdges, out var separated, admission: Reject));
        Assert.False(separated.HasValue);
    }

    [Fact]
    public void FilteredPenetration_WithoutAnyNonzeroAxis_ShouldPreserveNativeFailure()
    {
        var collapsed = new Vector3d[CubePoints.Length];
        Assert.False(WideOrientedBox.TryGetConvexHullPenetration(Vector3d.Zero, FixedQuaternion.Identity,
            collapsed, CubeTriangles, CubeEdges, Vector3d.Zero, FixedQuaternion.Identity,
            collapsed, CubeTriangles, CubeEdges, out var native));
        Assert.False(native.HasValue);
        Assert.False(WideOrientedBox.TryGetConvexHullPenetration(Vector3d.Zero, FixedQuaternion.Identity,
            collapsed, CubeTriangles, CubeEdges, Vector3d.Zero, FixedQuaternion.Identity,
            collapsed, CubeTriangles, CubeEdges, out var filtered,
            admission: static (in WidePointSpanPenetration _) => throw new InvalidOperationException("A zero axis cannot offer a signed exit.")));
        Assert.False(filtered.HasValue);
    }

    [Fact]
    public void PartlyExposedSupportEdge_ShouldNotAdmitWitnessesOnItsCoveredPortion()
    {
        Vector3d[] mover = Array.ConvertAll(CubePoints, point => point * Fixed64.Half);
        Vector3d center = new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero);
        Vector3d[] floor = { new(-4, 0, -4), new(0, 0, -4), new(0, 0, 4), new(-4, 0, 4) };
        int[] triangles = { 0, 1, 2, 0, 2, 3 }, edges = { 0, 1, 1, 2, 2, 3, 3, 0 };
        var domain = new PartlyExposedFloorDomain();
        int Visit(in WidePointSpanPenetration candidate) => WideOrientedBox.VisitConvexHullContacts(
            center, FixedQuaternion.Identity, mover, CubeTriangles, CubeEdges,
            Vector3d.Zero, FixedQuaternion.Identity, floor, triangles, edges, candidate, 0,
            static (_, _) => throw new InvalidOperationException("Admission must precede materialization."), domain, admissionOnly: true);
        bool Admit(in WidePointSpanPenetration candidate) => Visit(candidate) != 0;
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(center, FixedQuaternion.Identity,
            mover, CubeTriangles, CubeEdges, Vector3d.Zero, FixedQuaternion.Identity, floor, triangles, edges, out var native));
        Assert.True((native.Negate ? -native.Axis : native.Axis).X.Sign < 0);
        Assert.Equal(0, Visit(native));
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(center, FixedQuaternion.Identity,
            mover, CubeTriangles, CubeEdges, Vector3d.Zero, FixedQuaternion.Identity, floor, triangles, edges,
            out var admitted, admission: Admit));
        Assert.True(admitted.HasValue); Assert.True(Visit(admitted) > 0);
        var contacts = new System.Collections.Generic.List<FixedContactAnchors>();
        Assert.True(WideOrientedBox.VisitConvexHullContacts(center, FixedQuaternion.Identity, mover, CubeTriangles, CubeEdges,
            Vector3d.Zero, FixedQuaternion.Identity, floor, triangles, edges, admitted, contacts,
            static (pool, contact) => pool.Add(contact), domain) > 0);
        foreach (var contact in contacts)
        {
            Assert.Equal(Vector3d.Down, contact.Normal); Assert.Equal(Fixed64.Quarter, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out var first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out var second));
            Assert.Equal(-Fixed64.Quarter, first.Y); Assert.Equal(Fixed64.Zero, second.Y);
            Assert.True(first.X >= -Fixed64.Quarter && first.X <= Fixed64.Zero);
            Assert.Equal(first.X, second.X); Assert.Equal(first.Z, second.Z);
        }
    }

    [Fact]
    public void RetainedSourceEndsInsideASupportFace_ShouldRemainExactFiniteWitnesses()
    {
        Vector3d[] mover = Array.ConvertAll(CubePoints, point => point * Fixed64.Half);
        Vector3d center = new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero);
        Vector3d[] floor = { new(-4, 0, -4), new(0, 0, -4), new(0, 0, 4), new(-4, 0, 4) };
        int[] triangles = { 0, 1, 2, 0, 2, 3 }, edges = { 0, 1, 1, 2, 2, 3, 3, 0 };
        var domain = new PartlyExposedFloorDomain(retainMiddle: true);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(center, FixedQuaternion.Identity,
            mover, CubeTriangles, CubeEdges, Vector3d.Zero, FixedQuaternion.Identity, floor, triangles, edges, out var penetration));
        Assert.Equal(1, WideOrientedBox.VisitConvexHullContacts(center, FixedQuaternion.Identity, mover, CubeTriangles, CubeEdges,
            Vector3d.Zero, FixedQuaternion.Identity, floor, triangles, edges, penetration, 0,
            static (_, _) => throw new InvalidOperationException("Admission must not round a witness."), domain, admissionOnly: true));
        var contacts = new System.Collections.Generic.List<FixedContactAnchors>();
        Assert.True(WideOrientedBox.VisitConvexHullContacts(center, FixedQuaternion.Identity, mover, CubeTriangles, CubeEdges,
            Vector3d.Zero, FixedQuaternion.Identity, floor, triangles, edges, penetration, contacts,
            static (pool, contact) => pool.Add(contact), domain) >= 2);
        var ends = new System.Collections.Generic.HashSet<Fixed64>();
        foreach (var contact in contacts)
        {
            Assert.Equal(Vector3d.Left, contact.Normal); Assert.Equal(Fixed64.Quarter, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out var first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out var second));
            Assert.Equal(-Fixed64.Quarter, first.X); Assert.Equal(Fixed64.Zero, second.X);
            Assert.Equal(Fixed64.Zero, first.Y); Assert.Equal(Fixed64.Zero, second.Y);
            Assert.Equal(first.Z, second.Z); Assert.Equal(Fixed64.Quarter, second.Z.Abs()); ends.Add(second.Z);
        }
        Assert.Equal(2, ends.Count);
    }

    [Fact]
    public void NonparallelSupportEdges_ShouldApplyTheExactRetainedParameterBeforeRounding()
    {
        Vector3d[] first = { new(-1, 0, 0), new(1, 0, 0), new(0, 1, -1) };
        Vector3d[] second = { new(0, -1, 0), new(0, 1, 0), new(1, 0, 1) };
        int[] triangles = { 0, 1, 2 }, edges = { 0, 1, 1, 2, 2, 0 };
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(Vector3d.Zero, FixedQuaternion.Identity,
            first, triangles, edges, Vector3d.Zero, FixedQuaternion.Identity, second, triangles, edges,
            out var penetration, admission: static (in WidePointSpanPenetration candidate) =>
            {
                WideAxis3 normal = candidate.Negate ? -candidate.Axis : candidate.Axis;
                return normal.X.IsZero && normal.Y.IsZero && normal.Z.Sign > 0;
            }));
        var contacts = new System.Collections.Generic.List<FixedContactAnchors>();
        Assert.Equal(1, WideOrientedBox.VisitConvexHullContacts(Vector3d.Zero, FixedQuaternion.Identity, first, triangles, edges,
            Vector3d.Zero, FixedQuaternion.Identity, second, triangles, edges, penetration, contacts,
            static (pool, contact) => pool.Add(contact)));
        var contact = Assert.Single(contacts);
        Assert.Equal(Fixed64.Zero, contact.Depth); Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out var a)); Assert.Equal(Vector3d.Zero, a);
        Assert.True(contact.SecondAnchor.TryGetPoint(out var b)); Assert.Equal(a, b);
        var domain = new RetainedEdgeQuarterDomain();
        contacts.Clear();
        Assert.Equal(0, WideOrientedBox.VisitConvexHullContacts(Vector3d.Zero, FixedQuaternion.Identity, first, triangles, edges,
            Vector3d.Zero, FixedQuaternion.Identity, second, triangles, edges, penetration, contacts,
            static (pool, value) => pool.Add(value), domain));
        Assert.Empty(contacts); Assert.True(domain.RejectedMiddle);
        Assert.Equal(0, WideOrientedBox.VisitConvexHullContacts(Vector3d.Zero, FixedQuaternion.Identity, first, triangles, edges,
            Vector3d.Zero, FixedQuaternion.Identity, second, triangles, edges, penetration, 0,
            static (_, _) => throw new InvalidOperationException("Admission must not materialize an edge foot."), domain, admissionOnly: true));
        Assert.Equal(1, WideOrientedBox.VisitConvexHullContacts(Vector3d.Zero, FixedQuaternion.Identity, first, triangles, edges,
            Vector3d.Zero, FixedQuaternion.Identity, second, triangles, edges, penetration, 0,
            static (_, _) => throw new InvalidOperationException("Admission must not materialize an edge foot."), admissionOnly: true));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ExcludedFiniteSupport_ShouldEmitNeitherVerticesNorProjectedEdgeFeet(bool crossed, bool emptyEdges)
    {
        // Contained faces offer support vertices. Crossed thin faces have no
        // contained corners: their finite patch is supplied by interior edge feet.
        Vector3d[] first = crossed ? Array.ConvertAll(CubePoints,
            point => new Vector3d(point.X, point.Y * 2, point.Z * Fixed64.Quarter)) : CubePoints;
        Vector3d[] second = Array.ConvertAll(CubePoints, point => crossed
            ? new Vector3d(point.X, point.Y * Fixed64.Quarter, point.Z * 2)
            : new Vector3d(point.X, point.Y * 2, point.Z * 2));
        Vector3d offset = Vector3d.Right * (Fixed64.One + Fixed64.Half);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(Vector3d.Zero, FixedQuaternion.Identity,
            first, CubeTriangles, CubeEdges, offset, FixedQuaternion.Identity,
            second, CubeTriangles, CubeEdges, out var penetration));
        var contacts = new System.Collections.Generic.List<FixedContactAnchors>();
        Assert.True(WideOrientedBox.VisitConvexHullContacts(Vector3d.Zero, FixedQuaternion.Identity,
            first, CubeTriangles, CubeEdges, offset, FixedQuaternion.Identity, second, CubeTriangles, CubeEdges,
            penetration, contacts, static (pool, contact) => pool.Add(contact)) > 0);
        foreach (var contact in contacts)
        {
            Assert.Equal(Vector3d.Right, contact.Normal); Assert.Equal(Fixed64.Half, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out var point));
            Assert.Equal(Fixed64.One, point.X);
            Assert.Equal(crossed ? Fixed64.Quarter : Fixed64.One, point.Y.Abs());
            Assert.Equal(crossed ? Fixed64.Quarter : Fixed64.One, point.Z.Abs());
        }
        var domain = new ExcludedSupportDomain(emptyEdges);
        // A broad retained interval can still be declined at its exact finite
        // foot; it must agree with excluding the complete source edge up front.
        foreach (bool admissionOnly in new[] { false, true })
            Assert.Equal(0, WideOrientedBox.VisitConvexHullContacts(Vector3d.Zero, FixedQuaternion.Identity,
                first, CubeTriangles, CubeEdges, offset, FixedQuaternion.Identity, second, CubeTriangles, CubeEdges,
                penetration, 0, static (_, _) => throw new InvalidOperationException("An excluded support cannot become a witness."),
                domain, admissionOnly));
    }

    private sealed class ExcludedSupportDomain : IConvexHullContactDomain
    {
        private static readonly RationalSegmentInterval[] Full = { default };
        private readonly bool _emptyEdges;
        internal ExcludedSupportDomain(bool emptyEdges) => _emptyEdges = emptyEdges;
        public ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first, int start, int end, in WideAxis3 normal) =>
            _emptyEdges ? ReadOnlySpan<RationalSegmentInterval>.Empty : Full;
        public bool ContainsPoint(bool first, int start, int end, in Signed832 numerator, in Signed832 denominator, in WideAxis3 normal) => false;
    }

    private sealed class RetainedEdgeQuarterDomain : IConvexHullContactDomain
    {
        private static readonly RationalSegmentInterval[] Full = { default };
        private static readonly RationalSegmentInterval[] Quarter = { new(Number(0), Number(1), Number(1), Number(4)) };
        internal bool RejectedMiddle { get; private set; }
        public ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first, int start, int end, in WideAxis3 normal) => first ? Full : Quarter;
        public bool ContainsPoint(bool first, int start, int end, in Signed832 numerator, in Signed832 denominator, in WideAxis3 normal)
        {
            if (first) return true;
            if (start == end) return start == 0;
            Assert.Equal(0, start); Assert.Equal(1, end);
            Assert.Equal(0, WideArithmetic.CompareNonNegativeProducts(numerator, Signed832.ExtendValue(Signed192.Signed(2)),
                denominator, Signed832.ExtendValue(Signed192.Signed(1))));
            RejectedMiddle = true;
            return WideArithmetic.CompareNonNegativeProducts(numerator, Signed832.ExtendValue(Signed192.Signed(4)),
                denominator, Signed832.ExtendValue(Signed192.Signed(1))) <= 0;
        }
        private static Signed320 Number(long value) => Signed320.ExtendValue(Signed192.Signed(value));
    }

    private sealed class PartlyExposedFloorDomain : IConvexHullContactDomain
    {
        private static readonly RationalSegmentInterval[] Full = { default };
        private static readonly RationalSegmentInterval[] End = { new(Number(3), Number(4), Number(1), Number(1)) };
        private static readonly RationalSegmentInterval[] Start = { new(Number(0), Number(1), Number(1), Number(4)) };
        private static readonly RationalSegmentInterval[] Middle = { new(Number(15), Number(32), Number(17), Number(32)) };
        private readonly bool _retainMiddle;
        internal PartlyExposedFloorDomain(bool retainMiddle = false) => _retainMiddle = retainMiddle;
        public ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first, int start, int end, in WideAxis3 normal) =>
            first || !((start == 1 && end == 2) || (start == 2 && end == 1)) ? Full : _retainMiddle ? Middle : start == 1 ? End : Start;
        public bool ContainsPoint(bool first, int start, int end, in Signed832 numerator, in Signed832 denominator, in WideAxis3 normal)
        {
            if (first || start == end) return true;
            ref readonly RationalSegmentInterval clip = ref GetEdgeIntervals(first, start, end, normal)[0];
            if (!clip.IsClipped) return true;
            return WideArithmetic.CompareNonNegativeProducts(numerator, Signed832.ExtendValue(Signed576.ExtendValue(clip.LowerDenominator)),
                    Signed832.ExtendValue(Signed576.ExtendValue(clip.LowerNumerator)), denominator) >= 0
                && WideArithmetic.CompareNonNegativeProducts(numerator, Signed832.ExtendValue(Signed576.ExtendValue(clip.UpperDenominator)),
                    Signed832.ExtendValue(Signed576.ExtendValue(clip.UpperNumerator)), denominator) <= 0;
        }
        private static Signed320 Number(long value) => Signed320.ExtendValue(Signed192.Signed(value));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LowerDimensionalHullWithIndependentRigidFrames_ShouldPreserveItsFiniteCapsuleExit(bool line)
    {
        Vector3d[] points = line
            ? new[] { Vector3d.Backward, Vector3d.Zero, Vector3d.Forward }
            : new[] { new Vector3d(0, -1, -1), new Vector3d(0, 1, -1),
                new Vector3d(0, 1, 1), new Vector3d(0, -1, 1) };
        int[] triangles = line ? new[] { 0, 1, 2 } : new[] { 0, 1, 2, 0, 2, 3 };
        int[] edges = line ? new[] { 0, 1, 1, 2, 2, 0 } : new[] { 0, 1, 1, 2, 2, 3, 3, 0 };
        // The planar case retains a zero X radial component. The line retains
        // zero X/Y components. Both require exact later components across
        // independent rational frames, rather than assuming the first fits.
        FixedQuaternion hullRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            line ? Fixed64.Zero : (Fixed64)17, Fixed64.Zero, line ? (Fixed64)17 : Fixed64.Zero);
        FixedQuaternion capsuleRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            line ? Fixed64.Zero : (Fixed64)41, Fixed64.Zero, line ? (Fixed64)41 : Fixed64.Zero);
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(Vector3d.Zero, hullRotation,
            points, triangles, edges, Vector3d.Zero, capsuleRotation, Vector3d.Up,
            Fixed64.One, Fixed64.Quarter, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.InRange(Vector3d.Cross(first - second, contact.Normal).Magnitude.m_rawValue, 0L, 32L);
        if (!line) Assert.Equal(Fixed64.One, FixedMath.Abs(contact.Normal.X));
    }

    [Fact]
    public void ContainedCapsuleWithIndependentRigidFrames_ShouldKeepItsFullWidthExit()
    {
        FixedQuaternion hullRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)31);
        FixedQuaternion capsuleRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)13, (Fixed64)41, (Fixed64)7);
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(Vector3d.Zero, hullRotation,
            CubePoints, CubeTriangles, CubeEdges, Vector3d.Zero, capsuleRotation, Vector3d.Up,
            Fixed64.One, Fixed64.Quarter, out FixedContactAnchors contact));
        // The unit cube contains the entire capsule. Its exit must include the
        // radius and a finite core support, regardless of relative frame scale.
        Assert.InRange(contact.Depth.m_rawValue, (Fixed64.One + Fixed64.Quarter).m_rawValue,
            (Fixed64.One + Fixed64.FromFraction(3, 4)).m_rawValue);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.InRange(Vector3d.Cross(first - second, contact.Normal).Magnitude.m_rawValue, 0L, 32L);
        Vector3d localExit = capsuleRotation.Inverse().Rotate(second);
        Fixed64 beyondCore = FixedMath.Max(Fixed64.Zero, FixedMath.Abs(localExit.Y) - Fixed64.Half);
        Fixed64 radialSquared = localExit.X * localExit.X + localExit.Z * localExit.Z + beyondCore * beyondCore;
        Assert.InRange(FixedMath.Abs(radialSquared - Fixed64.Quarter * Fixed64.Quarter).m_rawValue, 0L, 16L);
    }

    private static readonly Vector3d[] CubePoints =
    {
        new(-1, -1, -1),
        new(1, -1, -1),
        new(1, 1, -1),
        new(-1, 1, -1),
        new(-1, -1, 1),
        new(1, -1, 1),
        new(1, 1, 1),
        new(-1, 1, 1)
    };

    private static readonly int[] CubeTriangles =
    {
        0, 2, 1, 0, 3, 2,
        4, 5, 6, 4, 6, 7,
        0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5,
        0, 1, 5, 0, 5, 4,
        3, 7, 6, 3, 6, 2
    };

    private static readonly int[] CubeEdges =
    {
        0, 1, 1, 2, 2, 3, 3, 0,
        4, 5, 5, 6, 6, 7, 7, 4,
        0, 4, 1, 5, 2, 6, 3, 7
    };

    [Fact]
    public void TryGetContact_ReturnsRigidLocalWitnesses()
    {
        Assert.True(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(new Vector3d(1, -1, -1), contact.FirstAnchor.LocalPoint);
        Assert.Equal(new Vector3d(-1, -1, -1), contact.SecondAnchor.LocalPoint);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.Equal(new Vector3d(
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero), separation);
    }

    [Theory]
    [InlineData(0, -1)]
    [InlineData(0, 1)]
    [InlineData(1, -1)]
    [InlineData(1, 1)]
    [InlineData(2, -1)]
    [InlineData(2, 1)]
    public void HullPenetration_RetainsExactDepthAndOrientedAxis(int axis, int sign)
    {
        Vector3d origin = Vector3d.Zero;
        origin[axis] = sign * Fixed64.FromFraction(3, 2);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            origin, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            out WidePointSpanPenetration penetration));

        WideAxis3 oriented = penetration.Negate ? -penetration.Axis : penetration.Axis;
        Vector3d normal = WideNormalization.GetNormalized(Signed576.ExtendValue(oriented.X),
            Signed576.ExtendValue(oriented.Y), Signed576.ExtendValue(oriented.Z));
        Vector3d expected = Vector3d.Zero;
        expected[axis] = (Fixed64)sign;
        Assert.Equal(expected, normal);
        Assert.Equal(Fixed64.Half, penetration.GetRoundedDepth(out bool clamped));
        Assert.False(clamped);

        origin[axis] = sign * (Fixed64)3;
        Assert.False(WideOrientedBox.TryGetConvexHullPenetration(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            origin, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            out penetration));
        Assert.False(penetration.HasValue);
    }

    [Fact]
    public void HullPenetration_RejectsEdgeCrossSeparationBetweenRotatedHulls()
    {
        Vector3d origin = new(Fixed64.FromFraction(-27, 10), Fixed64.FromFraction(-3, 2),
            Fixed64.FromFraction(-19, 10));
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)30, (Fixed64)45);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, ReadOnlySpan<int>.Empty,
            origin, rotation, CubePoints, CubeTriangles, ReadOnlySpan<int>.Empty, out _));
        Assert.False(WideOrientedBox.TryGetConvexHullPenetration(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            origin, rotation, CubePoints, CubeTriangles, CubeEdges, out WidePointSpanPenetration penetration));
        Assert.False(penetration.HasValue);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void HullPenetration_RedundantLocalDirectionsPreserveTheFirstExactAxis(bool filtered, bool centered)
    {
        int[] triangles = new int[3 + 2 * CubeTriangles.Length], edges = new int[2 + 2 * CubeEdges.Length];
        for (int index = 0; index < CubeTriangles.Length; index += 3)
        {
            int output = 3 + 2 * index;
            Array.Copy(CubeTriangles, index, triangles, output, 3);
            triangles[output + 3] = CubeTriangles[index + 2];
            triangles[output + 4] = CubeTriangles[index + 1];
            triangles[output + 5] = CubeTriangles[index];
        }
        for (int index = 0; index < CubeEdges.Length; index += 2)
        {
            int output = 2 + 2 * index;
            edges[output] = edges[output + 3] = CubeEdges[index];
            edges[output + 1] = edges[output + 2] = CubeEdges[index + 1];
        }
        FixedQuaternion firstRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)31),
            secondRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)7, (Fixed64)11, (Fixed64)19);
        Vector3d offset = centered ? Vector3d.Zero : new(Fixed64.One, Fixed64.Quarter, Fixed64.FromFraction(1, 8));
        PointSpanPenetrationAdmission? admission = filtered ? static (in WidePointSpanPenetration _) => true : null;
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(
            Vector3d.Zero, firstRotation, CubePoints, CubeTriangles, CubeEdges,
            offset, secondRotation, CubePoints, CubeTriangles, CubeEdges, out WidePointSpanPenetration expected, admission: admission));
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(
            Vector3d.Zero, firstRotation, CubePoints, triangles, edges,
            offset, secondRotation, CubePoints, triangles, edges, out WidePointSpanPenetration actual, skipParallelDirections: true, admission: admission));
        Assert.Equal(expected.Axis.X, actual.Axis.X);
        Assert.Equal(expected.Axis.Y, actual.Axis.Y);
        Assert.Equal(expected.Axis.Z, actual.Axis.Z);
        Assert.Equal(expected.Negate, actual.Negate);
        Assert.Equal(expected.ExactOverlap, actual.ExactOverlap);
        Assert.Equal(expected.ExactSquaredAxisLength, actual.ExactSquaredAxisLength);
        Assert.Equal(expected.ExactCommonDenominator, actual.ExactCommonDenominator);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HullContacts_EdgeCrossWinnerRetainsMatchedFiniteSupportEdges(bool reverse)
    {
        Vector3d offset = new(Fixed64.FromFraction(-24, 10), -Fixed64.One, Fixed64.FromFraction(-12, 10));
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)30, (Fixed64)45);
        Vector3d firstOrigin = reverse ? offset : Vector3d.Zero, secondOrigin = reverse ? Vector3d.Zero : offset;
        FixedQuaternion firstRotation = reverse ? rotation : FixedQuaternion.Identity,
            secondRotation = reverse ? FixedQuaternion.Identity : rotation;
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(
            firstOrigin, firstRotation, CubePoints, CubeTriangles, CubeEdges,
            secondOrigin, secondRotation, CubePoints, CubeTriangles, CubeEdges, out WidePointSpanPenetration penetration));
        var contacts = new System.Collections.Generic.List<FixedContactAnchors>();
        int count = WideOrientedBox.VisitConvexHullContacts(
            firstOrigin, firstRotation, CubePoints, CubeTriangles, CubeEdges,
            secondOrigin, secondRotation, CubePoints, CubeTriangles, CubeEdges,
            penetration, contacts, static (list, contact) => list.Add(contact));
        Assert.Equal(1, count);
        FixedContactAnchors contact = Assert.Single(contacts);
        Assert.Equal(penetration.GetRoundedDepth(out bool clamped), contact.Depth);
        Assert.False(clamped);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d gap));
        Assert.InRange(Vector3d.Cross(gap, contact.Normal).Magnitude.m_rawValue, 0L, 64L);
        Assert.InRange(FixedMath.Abs(Vector3d.Dot(gap, contact.Normal) - contact.Depth).m_rawValue, 0L, 64L);
        AssertEdge(contact.FirstAnchor.LocalPoint); AssertEdge(contact.SecondAnchor.LocalPoint);

        static void AssertEdge(Vector3d point)
        {
            int boundaryCoordinates = 0;
            for (int axis = 0; axis < 3; axis++)
            {
                Assert.InRange(point[axis].m_rawValue, -Fixed64.One.m_rawValue, Fixed64.One.m_rawValue);
                if (FixedMath.Abs(point[axis]) == Fixed64.One) boundaryCoordinates++;
            }
            Assert.Equal(2, boundaryCoordinates);
        }
    }

    [Fact]
    public void ClosestNonparallelSegmentParameters_RejectsParallelSegments()
    {
        var segment = new FixedSegment(Vector3d.Left, Vector3d.Right);
        Assert.False(WideTriangleRelations.TryGetClosestNonparallelSegmentParameters(
            Vector3d.Zero, FixedQuaternion.Identity, segment,
            Vector3d.Up, FixedQuaternion.Identity, segment, out _, out _, out _, out _));
    }

    [Theory]
    [InlineData(-2, 0)]
    [InlineData(2, 0)]
    [InlineData(0, 2)]
    [InlineData(0, -2)]
    public void ClosestNonparallelSegmentParameters_RejectsLineFeetOutsideFiniteSegments(int x, int z)
    {
        Assert.False(WideTriangleRelations.TryGetClosestNonparallelSegmentParameters(
            Vector3d.Zero, FixedQuaternion.Identity, new FixedSegment(Vector3d.Left, Vector3d.Right),
            new Vector3d(x, 1, z), FixedQuaternion.Identity, new FixedSegment(Vector3d.Backward, Vector3d.Forward),
            out _, out _, out _, out _));
    }

    [Fact]
    public void HullContacts_VertexTouchWithoutFaceOrEdgeWitnessReturnsNoContacts()
    {
        Vector3d[] first = { new(-1, -1, -1), new(1, -1, -1), new(0, 1, -1), new(0, 0, 1) };
        Vector3d[] second = Array.ConvertAll(first, point => new Vector3d(point.X, point.Y, -point.Z));
        int[] triangles = { 0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3 };
        int[] edges = { 0, 1, 1, 2, 2, 0, 0, 3, 1, 3, 2, 3 };
        Vector3d offset = Vector3d.Forward * 2;
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(
            Vector3d.Zero, FixedQuaternion.Identity, first, triangles, edges,
            offset, FixedQuaternion.Identity, second, triangles, edges, out WidePointSpanPenetration penetration));
        Assert.Equal(Fixed64.Zero, penetration.GetRoundedDepth(out _));
        var contacts = new System.Collections.Generic.List<FixedContactAnchors>();
        Assert.Equal(0, WideOrientedBox.VisitConvexHullContacts(
            Vector3d.Zero, FixedQuaternion.Identity, first, triangles, edges,
            offset, FixedQuaternion.Identity, second, triangles, edges,
            penetration, contacts, static (list, contact) => list.Add(contact)));
        Assert.Empty(contacts);
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, 17)]
    [InlineData(true, 17)]
    public void HullFaceContacts_MatchFiniteFaceProjectionsInBothOrders(bool reverse, int angle)
    {
        Vector3d[] wideCube = Array.ConvertAll(CubePoints, point => new Vector3d(point.X, point.Y * 2, point.Z * 2));
        Vector3d offset = new(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)angle, Fixed64.Zero, Fixed64.Zero);
        Vector3d firstOrigin = reverse ? offset : Vector3d.Zero, secondOrigin = reverse ? Vector3d.Zero : offset;
        FixedQuaternion firstRotation = reverse ? rotation : FixedQuaternion.Identity,
            secondRotation = reverse ? FixedQuaternion.Identity : rotation;
        Vector3d[] firstPoints = reverse ? wideCube : CubePoints, secondPoints = reverse ? CubePoints : wideCube;
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(
            firstOrigin, firstRotation, firstPoints, CubeTriangles, CubeEdges,
            secondOrigin, secondRotation, secondPoints, CubeTriangles, CubeEdges,
            out WidePointSpanPenetration penetration));
        var contacts = new System.Collections.Generic.List<FixedContactAnchors>();
        int count = WideOrientedBox.VisitConvexHullContacts(
            firstOrigin, firstRotation, firstPoints, CubeTriangles, CubeEdges,
            secondOrigin, secondRotation, secondPoints, CubeTriangles, CubeEdges, penetration, contacts, static (list, contact) => list.Add(contact));

        Assert.Equal(count, contacts.Count);
        if (!reverse) Assert.Equal(4, count);
        var unique = new System.Collections.Generic.HashSet<Vector3d>();
        for (int index = 0; index < count; index++)
        {
            FixedContactAnchors contact = contacts[index];
            Assert.Equal(reverse ? Vector3d.Left : Vector3d.Right, contact.Normal);
            Assert.Equal(Fixed64.Half, contact.Depth);
            Assert.False(contact.DepthIsClamped);
            Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d separation));
            Assert.InRange(Vector3d.Cross(separation, contact.Normal).Magnitude.m_rawValue, 0L, 32L);
            Assert.Equal(reverse ? -Fixed64.Half : Fixed64.Half, separation.X);
            FixedPointAnchor cubeAnchor = reverse ? contact.SecondAnchor : contact.FirstAnchor;
            unique.Add(cubeAnchor.LocalPoint);
            Assert.Equal(Fixed64.One, cubeAnchor.LocalPoint.X);
            Assert.Equal(Fixed64.One, FixedMath.Abs(cubeAnchor.LocalPoint.Y));
            Assert.Equal(Fixed64.One, FixedMath.Abs(cubeAnchor.LocalPoint.Z));
        }
        Assert.Equal(4, unique.Count);
    }

    [Fact]
    public void HullFaceContacts_RetainTheCompleteCrossedFacePatch()
    {
        Vector3d[] firstPoints = Array.ConvertAll(CubePoints,
            point => new Vector3d(point.X, point.Y * 2, point.Z * Fixed64.Quarter));
        Vector3d[] secondPoints = Array.ConvertAll(CubePoints,
            point => new Vector3d(point.X, point.Y * Fixed64.Quarter, point.Z * 2));
        Vector3d offset = new(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero);
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(
            Vector3d.Zero, FixedQuaternion.Identity, firstPoints, CubeTriangles, CubeEdges,
            offset, FixedQuaternion.Identity, secondPoints, CubeTriangles, CubeEdges,
            out WidePointSpanPenetration penetration));
        var contacts = new System.Collections.Generic.List<FixedContactAnchors>();
        int count = WideOrientedBox.VisitConvexHullContacts(
            Vector3d.Zero, FixedQuaternion.Identity, firstPoints, CubeTriangles, CubeEdges,
            offset, FixedQuaternion.Identity, secondPoints, CubeTriangles, CubeEdges, penetration, contacts, static (list, contact) => list.Add(contact));
        var corners = new System.Collections.Generic.HashSet<Vector3d>();
        for (int index = 0; index < count; index++)
        {
            Assert.True(contacts[index].FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contacts[index].SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), first - second);
            Assert.Equal(Fixed64.One, first.X);
            Assert.Equal(Fixed64.Quarter, FixedMath.Abs(first.Y));
            Assert.Equal(Fixed64.Quarter, FixedMath.Abs(first.Z));
            corners.Add(first);
        }
        Assert.Equal(4, corners.Count);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HullFaceContacts_CompletePartiallyClippedFacesWithoutTriangleDiagonalCuts(bool reverse)
    {
        Vector3d[] wall = Array.ConvertAll(CubePoints,
            point => new Vector3d(point.X, point.Y * 2, point.Z * Fixed64.Half));
        Vector3d offset = new(Fixed64.FromFraction(3, 2), Fixed64.Zero, -Fixed64.Half);
        Vector3d firstOrigin = reverse ? offset : Vector3d.Zero, secondOrigin = reverse ? Vector3d.Zero : offset;
        Vector3d[] firstPoints = reverse ? wall : CubePoints, secondPoints = reverse ? CubePoints : wall;
        Assert.True(WideOrientedBox.TryGetConvexHullPenetration(
            firstOrigin, FixedQuaternion.Identity, firstPoints, CubeTriangles, CubeEdges,
            secondOrigin, FixedQuaternion.Identity, secondPoints, CubeTriangles, CubeEdges,
            out WidePointSpanPenetration penetration));
        var contacts = new System.Collections.Generic.List<FixedContactAnchors>();
        int count = WideOrientedBox.VisitConvexHullContacts(
            firstOrigin, FixedQuaternion.Identity, firstPoints, CubeTriangles, CubeEdges,
            secondOrigin, FixedQuaternion.Identity, secondPoints, CubeTriangles, CubeEdges, penetration, contacts, static (list, contact) => list.Add(contact));
        var corners = new System.Collections.Generic.HashSet<Vector3d>();
        for (int index = 0; index < count; index++)
        {
            FixedContactAnchors contact = contacts[index];
            Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d separation));
            Assert.Equal(new Vector3d(reverse ? -Fixed64.Half : Fixed64.Half, Fixed64.Zero, Fixed64.Zero), separation);
            FixedPointAnchor cubeAnchor = reverse ? contact.SecondAnchor : contact.FirstAnchor;
            Assert.True(cubeAnchor.TryGetPoint(out Vector3d corner));
            Assert.Equal(Fixed64.One, corner.X);
            Assert.Equal(Fixed64.One, FixedMath.Abs(corner.Y));
            Assert.True(corner.Z == -Fixed64.One || corner.Z == Fixed64.Zero);
            corners.Add(corner);
        }
        Assert.Equal(4, corners.Count);
    }

    [Fact]
    public void TryGetContact_PreservesWitnessWhenAbsolutePointIsUnavailable()
    {
        Vector3d firstOrigin = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4),
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d secondOrigin = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedConvexHullRelations.TryGetContact(
            firstOrigin,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            secondOrigin,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));

        Assert.False(contact.FirstAnchor.TryGetPoint(out _));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.Equal(new Vector3d(
            Fixed64.FromFraction(7, 4),
            Fixed64.Zero,
            Fixed64.Zero), separation);
    }

    [Fact]
    public void TryGetContact_RejectsFaceAndEdgeSeparations()
    {
        Assert.False(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(3, 0, 0),
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out _));

        Assert.False(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAngles(
                Fixed64.PiOver6,
                Fixed64.PiOver4,
                Fixed64.Zero),
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(3, 3, 3),
            FixedQuaternion.FromEulerAngles(
                -Fixed64.PiOver4,
                Fixed64.PiOver6,
                Fixed64.PiOver4),
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out _));
    }

    [Fact]
    public void TryGetContact_RejectsEdgeCrossSeparationBetweenRotatedHulls()
    {
        Assert.False(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.FromFraction(-27, 10),
                Fixed64.FromFraction(-3, 2),
                Fixed64.FromFraction(-19, 10)),
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)15,
                (Fixed64)30,
                (Fixed64)45),
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out _));
    }

    [Fact]
    public void TryGetContact_OrientsMirroredOverlapTowardTheSecondHull()
    {
        Assert.True(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.FromFraction(-3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));

        Assert.Equal(-Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_ReturnsExactRigidFrameContact()
    {
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.One, contact.FirstAnchor.LocalPoint.X);
        Assert.Equal(-Fixed64.One, contact.SecondAnchor.LocalDisplacement.X);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_DistinguishesScalarFaceTangency()
    {
        Vector3d hullOrigin = new(
            Fixed64.MaxValue - (Fixed64)2,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d capsuleCenter = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            hullOrigin,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d hullPoint));
        Assert.Equal(
            Fixed64.MaxValue - Fixed64.One,
            hullPoint.X);

        bool hasMiss = FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            hullOrigin,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.FromRaw(Fixed64.One.m_rawValue - 1L),
            out FixedContactAnchors missContact);
        Assert.False(
            hasMiss,
            $"normal={missContact.Normal}, depthRaw={missContact.Depth.m_rawValue}");
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_PreservesUnrepresentableCapsuleWitness()
    {
        var points = new Vector3d[CubePoints.Length];
        for (int index = 0; index < points.Length; index++)
        {
            points[index] = new Vector3d(
                CubePoints[index].X * Fixed64.FromFraction(1, 4),
                CubePoints[index].Y * (Fixed64)2,
                CubePoints[index].Z * (Fixed64)2);
        }

        Vector3d hullOrigin = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4),
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d capsuleCenter = new(
            Fixed64.MaxValue - Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            hullOrigin,
            FixedQuaternion.Identity,
            points,
            CubeTriangles,
            CubeEdges,
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(-Vector3d.Right, contact.Normal);
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.Equal(contact.Depth, Vector3d.Dot(separation, contact.Normal));
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_ContainsAngledCore()
    {
        var points = new Vector3d[CubePoints.Length];
        for (int index = 0; index < points.Length; index++)
        {
            points[index] = new Vector3d(
                CubePoints[index].X * Fixed64.Half,
                CubePoints[index].Y * Fixed64.Half,
                CubePoints[index].Z * (Fixed64)2);
        }
        Vector3d segmentDirection = new(
            Fixed64.FromFraction(4, 5),
            Fixed64.FromFraction(3, 5),
            Fixed64.Zero);
        Vector3d rotationAxis =
            Vector3d.Cross(Vector3d.Up, segmentDirection);
        FixedQuaternion rotation = new FixedQuaternion(
            rotationAxis.X,
            rotationAxis.Y,
            rotationAxis.Z,
            Fixed64.One
            + Vector3d.Dot(Vector3d.Up, segmentDirection)).Normalized;

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            points,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.Zero,
                -Fixed64.FromFraction(1, 10),
                Fixed64.Zero),
            rotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.FromFraction(1, 10),
            out FixedContactAnchors contact));

        Assert.True(contact.Depth > Fixed64.Zero);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_UsesOpenPlanarHullFace()
    {
        Vector3d[] points =
        {
            new(-1, 0, -1),
            new(1, 0, -1),
            new(1, 0, 1),
            new(-1, 0, 1),
        };
        int[] triangles = { 0, 2, 1, 0, 3, 2 };
        int[] edges = { 0, 1, 1, 2, 2, 3, 3, 0 };

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            points,
            triangles,
            edges,
            Vector3d.Up,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            points,
            triangles,
            edges,
            Vector3d.Up,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One - Fixed64.FromRaw(1),
            out _));
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_UsesClosestEdgeAxis()
    {
        FixedQuaternion capsuleRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                Fixed64.Zero,
                (Fixed64)(-90));

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(7, 5),
                Fixed64.FromFraction(7, 5)),
            capsuleRotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out _));

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(27, 20),
                Fixed64.FromFraction(27, 20)),
            capsuleRotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.Normal.Y > Fixed64.Zero);
        Assert.True(contact.Normal.Z > Fixed64.Zero);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_RejectsEdgeCrossSeparation()
    {
        Vector3d capsuleAxis = new(1, 1, 1);

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.Zero,
                -Fixed64.FromFraction(13, 10),
                Fixed64.FromFraction(13, 10)),
            WideGeometry.GetCanonicalAxisRotation(capsuleAxis),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.FromFraction(1, 10),
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_RejectsEndpointCornerSeparation()
    {
        var halfCube = new Vector3d[CubePoints.Length];
        for (int index = 0; index < halfCube.Length; index++)
            halfCube[index] = CubePoints[index] * Fixed64.Half;

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(5, 2)),
            FixedQuaternion.Identity,
            halfCube,
            CubeTriangles,
            CubeEdges,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Forward,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_RejectsEndpointToEdgeSeparation()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Vector3d axis = new(Fixed64.Zero, diagonal, diagonal);
        Vector3d endpoint = new(
            Fixed64.Zero,
            Fixed64.FromFraction(7, 5),
            Fixed64.FromFraction(7, 5));
        Vector3d center = endpoint + axis;

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            center,
            WideGeometry.GetCanonicalAxisRotation(axis),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_RejectsAConcentricDegenerateHull()
    {
        Vector3d[] pointHull =
        {
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
        };
        int[] triangle = { 0, 1, 2 };
        int[] edges = { 0, 1, 1, 2, 2, 0 };

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            pointHull,
            triangle,
            edges,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_DistinguishesMaximumDepthFromOverflow()
    {
        var exactMaximumHull = new Vector3d[CubePoints.Length];
        var overflowingHull = new Vector3d[CubePoints.Length];
        Fixed64 exactExtent = Fixed64.MaxValue - Fixed64.One;
        for (int index = 0; index < CubePoints.Length; index++)
        {
            exactMaximumHull[index] = CubePoints[index] * exactExtent;
            overflowingHull[index] = CubePoints[index] * Fixed64.MaxValue;
        }

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            exactMaximumHull,
            CubeTriangles,
            CubeEdges,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors exactMaximum));
        Assert.Equal(Fixed64.MaxValue, exactMaximum.Depth);
        Assert.False(exactMaximum.DepthIsClamped);

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            overflowingHull,
            CubeTriangles,
            CubeEdges,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors overflow));
        Assert.Equal(Fixed64.MaxValue, overflow.Depth);
        Assert.True(overflow.DepthIsClamped);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_RoundsSubLatticeCornerDepth()
    {
        Fixed64 oneRaw = Fixed64.MinIncrement;

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.One + oneRaw,
                Fixed64.One + oneRaw,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.FromRaw(2),
            out FixedContactAnchors contact));

        Assert.Equal(oneRaw, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(new Vector3d(1, 1, 0), contact.FirstAnchor.LocalPoint);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_IsDirtyStackDeterministicAndAllocationFree()
    {
        Assert.True(TryGetExtremeCapsuleContact(
            out FixedContactAnchors expected));
        for (int iteration = 0; iteration < 8; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.True(TryGetExtremeCapsuleContact(
                out FixedContactAnchors actual));
            Assert.Equal(expected, actual);
        }

        _ = TryGetExtremeCapsuleContact(out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        int contacts = 0;
        for (int iteration = 0; iteration < 8; iteration++)
        {
            if (TryGetExtremeCapsuleContact(out _))
                contacts++;
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(8, contacts);
        Assert.Equal(before, after);
    }

    [Fact]
    public void TryGetContact_RejectsHullsWithoutAUsableSeparatingAxis()
    {
        Vector3d[] pointHull =
        {
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero
        };
        int[] triangle = { 0, 1, 2 };
        int[] edges = { 0, 1, 1, 2, 2, 0 };

        Assert.False(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            pointHull,
            triangle,
            edges,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            pointHull,
            triangle,
            edges,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ContainsPoint_UsesExactRigidFrames()
    {
        Vector3d origin = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.PiOver4);

        Assert.True(FixedConvexHullRelations.ContainsPoint(
            origin,
            rotation,
            CubePoints,
            CubeTriangles,
            Vector3d.Zero,
            new FixedPointAnchor(
                origin,
                rotation,
                new Vector3d(
                    Fixed64.Half,
                    Fixed64.Zero,
                    Fixed64.Zero))));
        Assert.False(FixedConvexHullRelations.ContainsPoint(
            origin,
            rotation,
            CubePoints,
            CubeTriangles,
            Vector3d.Zero,
            new FixedPointAnchor(
                origin,
                rotation,
                new Vector3d(2, 0, 0))));
        Assert.False(FixedConvexHullRelations.ContainsPoint(
            origin,
            rotation,
            CubePoints,
            CubeTriangles,
            Vector3d.Zero,
            default));
    }

    [Fact]
    public void ContainsPoint_IgnoresNonBoundaryTriangleEntries()
    {
        int[] triangles = new int[CubeTriangles.Length + 6];
        triangles[0] = 0;
        triangles[1] = 0;
        triangles[2] = 0;
        triangles[3] = 0;
        triangles[4] = 6;
        triangles[5] = 1;
        CubeTriangles.CopyTo(triangles, 6);

        Assert.True(FixedConvexHullRelations.ContainsPoint(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            triangles,
            Vector3d.Zero,
            new FixedPointAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero)));
        Assert.False(FixedConvexHullRelations.ContainsPoint(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            triangles,
            Vector3d.Zero,
            new FixedPointAnchor(
                new Vector3d(2, 0, 0),
                FixedQuaternion.Identity,
                Vector3d.Zero)));
    }

    [Fact]
    public void Relations_RejectMalformedHullContracts()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.ContainsPoint(
                Vector3d.Zero,
                default,
                CubePoints,
                CubeTriangles,
                Vector3d.Zero,
                default));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.ContainsPoint(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints.AsSpan(0, 2),
                CubeTriangles,
                Vector3d.Zero,
                default));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.ContainsPoint(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                Array.Empty<int>(),
                Vector3d.Zero,
                default));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.ContainsPoint(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                new[] { 0, 1 },
                Vector3d.Zero,
                default));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvexHullRelations.ContainsPoint(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                new[] { 0, 1, CubePoints.Length },
                Vector3d.Zero,
                default));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                new[] { 0 },
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                Array.Empty<int>(),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                Array.Empty<int>(),
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                default,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                Array.Empty<int>(),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                default,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvexHullRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                -Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvexHullRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Right,
                Fixed64.One,
                -Fixed64.One,
                out _));
    }

    [Theory]
    [InlineData(1, 0, 0)]
    [InlineData(3, 0, 2)]
    [InlineData(5, 0, 2)]
    [InlineData(1, 2, 2)]
    [InlineData(3, 2, 4)]
    public void TryGetCenteredCapsuleContact_RoundsExactHalfRawExitToEven(
        long axisRaw, long radiusRaw, long depthRaw)
    {
        Vector3d[] points = { new(-1, 0, -1), new(1, 0, -1), new(1, 0, 1), new(-1, 0, 1) };
        int[] triangles = { 0, 1, 2, 0, 2, 3 };
        int[] edges = { 0, 1, 1, 2, 2, 3, 3, 0 };
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, points, triangles, edges,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.FromRaw(axisRaw), Fixed64.FromRaw(radiusRaw), out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromRaw(depthRaw), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(0, 0, true)]
    [InlineData(16, 0, true)]
    [InlineData(16, 16, true)]
    [InlineData(19, 20, true)]
    [InlineData(19, 21, false)]
    [InlineData(21, 21, false)]
    public void TryGetCenteredCapsuleContact_PreservesFinitePlanarSphereFeatures(
        int xSixteenths, int zSixteenths, bool expected)
    {
        Vector3d[] points = { new(-1, 0, -1), new(1, 0, -1), new(1, 0, 1), new(-1, 0, 1) };
        int[] triangles = { 0, 1, 2, 0, 2, 3 };
        // Repeated and zero-length edges cannot displace the nearest finite feature.
        int[] edges = { 0, 0, 0, 1, 1, 2, 2, 3, 3, 0, 0, 1 };
        Vector3d center = new(Fixed64.FromFraction(xSixteenths, 16), Fixed64.Zero,
            Fixed64.FromFraction(zSixteenths, 16));
        Assert.Equal(expected, FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, points, triangles, edges,
            center, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.FromFraction(5, 16),
            out FixedContactAnchors contact));
        if (expected)
        {
            Assert.False(contact.DepthIsClamped);
            Assert.Equal(xSixteenths <= 16 ? Fixed64.FromFraction(5, 16) : Fixed64.Zero, contact.Depth);
        }
    }

    [Theory]
    [InlineData(2, 0, 0, false)]
    [InlineData(5, 0, 0, false)]
    [InlineData(5, 5, 0, false)]
    [InlineData(5, 5, 5, false)]
    [InlineData(2, 0, 0, true)]
    [InlineData(5, 0, 0, true)]
    [InlineData(5, 5, 0, true)]
    [InlineData(5, 5, 5, true)]
    public void TryGetCenteredCapsuleContact_WholeSphereExitAgreesWithEquivalentOrientedBox(
        int xQuarters, int yQuarters, int zQuarters, bool rotated)
    {
        FixedQuaternion rotation = rotated
            ? FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver6)
            : FixedQuaternion.Identity;
        Vector3d origin = new(3, -2, 4);
        Vector3d center = origin + rotation.Rotate(new Vector3d(
            Fixed64.FromFraction(xQuarters, 4), Fixed64.FromFraction(yQuarters, 4), Fixed64.FromFraction(zQuarters, 4)));
        var box = new FixedOrientedBox(origin, rotation, Vector3d.One);
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            origin, rotation, CubePoints, CubeTriangles, CubeEdges,
            center, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.True(box.TryGetCenteredCapsuleContact(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Zero, Fixed64.Half, out FixedContactAnchors equivalent));
        // At this unit scale, rounding depth, a normalized direction and final
        // coordinate products leaves at most eight raw units of exit uncertainty.
        Fixed64 bound = Fixed64.FromRaw(8);
        Assert.True(FixedMath.Abs(contact.Depth - equivalent.Depth) <= bound);
        foreach (bool beyond in new[] { false, true })
        {
            Vector3d moved = center + contact.Normal * (contact.Depth + (beyond ? bound : -bound));
            Assert.Equal(!beyond, box.TryGetCenteredCapsuleContact(moved, FixedQuaternion.Identity,
                Vector3d.Up, Fixed64.Zero, Fixed64.Half, out _));
        }
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void TryGetCenteredCapsuleContact_DistinguishesRawFaceProjectionBoundary(long beyondRaw)
    {
        Vector3d origin = new(Fixed64.MaxValue - (Fixed64)2, Fixed64.MinValue + (Fixed64)2, Fixed64.Zero);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver6);
        Vector3d center = origin + rotation.Rotate(new Vector3d(
            Fixed64.FromFraction(5, 4), Fixed64.One + Fixed64.FromRaw(beyondRaw), Fixed64.Zero));
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            origin, rotation, CubePoints, CubeTriangles, CubeEdges,
            center, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.True(new FixedOrientedBox(origin, rotation, Vector3d.One)
            .TryGetCenteredCapsuleContact(center, FixedQuaternion.Identity, Vector3d.Up,
                Fixed64.Zero, Fixed64.Half, out FixedContactAnchors equivalent));
        Assert.Equal(equivalent.Depth, contact.Depth);
        Assert.Equal(equivalent.Normal, contact.Normal);
        Assert.Equal(beyondRaw > 0, contact.Normal.Y > Fixed64.Zero);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StrictSphereOverlap_PreservesZeroDistanceBoundaryFeatures(bool vertex)
    {
        Vector3d center = vertex ? CubePoints[0] : new Vector3d(Fixed64.Zero, -Fixed64.One, -Fixed64.One);
        Assert.True(WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            center, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.Quarter));
        Assert.False(WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            center, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_PreservesWidePlanarEdgeAxes()
    {
        Fixed64 extent = Fixed64.MaxValue - Fixed64.One;
        Vector3d[] points = { new(-extent, Fixed64.Zero, -extent), new(extent, Fixed64.Zero, -extent),
            new(extent, Fixed64.Zero, extent), new(-extent, Fixed64.Zero, extent) };
        int[] triangles = { 0, 1, 2, 0, 2, 3 };
        int[] edges = { 0, 1, 1, 2, 2, 3, 3, 0 };
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, points, triangles, edges,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.Equal(Fixed64.One, FixedMath.Abs(contact.Normal.Y));
        Assert.Equal(Fixed64.Zero, contact.Normal.X);
        Assert.Equal(Fixed64.Zero, contact.Normal.Z);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(-3)]
    [InlineData(3)]
    public void TryGetCenteredCapsuleContact_PreservesFiniteSupportWhenCoreCenterProjectsOutsideFace(int x)
    {
        Vector3d center = Vector3d.Right * x;
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            center, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)8, Fixed64.Quarter,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.One + Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d hullPoint));
        Assert.Equal((Fixed64)Math.Sign(x), hullPoint.X);
        Assert.True(FixedMath.Abs(hullPoint.Y) <= Fixed64.One);
        Assert.Equal(Fixed64.One, FixedMath.Abs(hullPoint.Z));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Assert.Equal(Vector3d.Zero, Vector3d.Cross(offset, contact.Normal));
        Vector3d moved = center + contact.Normal * (contact.Depth + Fixed64.MinIncrement);
        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            moved, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)8, Fixed64.Quarter, out _));
    }

    [Theory]
    [InlineData(-3, false)]
    [InlineData(3, false)]
    [InlineData(-3, true)]
    [InlineData(3, true)]
    public void TryGetCenteredCapsuleContact_MatchesCoreAgainstFiniteTriangularSupportFeatures(int x, bool edge)
    {
        Fixed64 height = edge ? Fixed64.One : (Fixed64)3;
        Vector3d[] points = { new(-Fixed64.One, -height, -Fixed64.One), new(Fixed64.One, -height, -Fixed64.One), new(Fixed64.Zero, height, -Fixed64.One),
            new(-Fixed64.One, -height, Fixed64.One), new(Fixed64.One, -height, Fixed64.One), new(Fixed64.Zero, height, Fixed64.One) };
        int[] triangles = { 0, 2, 1, 3, 4, 5, 0, 1, 4, 0, 4, 3,
            1, 2, 5, 1, 5, 4, 2, 0, 3, 2, 3, 5 };
        int[] edges = { 0, 1, 1, 2, 2, 0, 3, 4, 4, 5, 5, 3, 0, 3, 1, 4, 2, 5 };
        Vector3d center = new((Fixed64)x, Fixed64.Half, Fixed64.Zero);
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, points, triangles, edges,
            center, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)8, Fixed64.Quarter,
            out FixedContactAnchors contact));
        Assert.Equal((edge ? Fixed64.Half : Fixed64.One) + Fixed64.Quarter, contact.Depth);
        Assert.Equal(Fixed64.One, FixedMath.Abs(edge ? contact.Normal.Y : contact.Normal.Z));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        // Admission is exact; the rational face intersection is rounded only when
        // its two local witnesses are materialized, allowing a few raw units.
        Vector3d tangent = Vector3d.Cross(offset, contact.Normal);
        Assert.True(FixedMath.Abs(tangent.X) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Y) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Z) <= Fixed64.FromRaw(16));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d hullPoint));
        Assert.True(FixedMath.Abs(hullPoint.Y - (edge ? Fixed64.One : Fixed64.Half)) <= Fixed64.FromRaw(2));
        Assert.True(hullPoint.X >= -Fixed64.Half && hullPoint.X <= Fixed64.Half);
        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, points, triangles, edges,
            center + contact.Normal * (contact.Depth + Fixed64.MinIncrement),
            FixedQuaternion.Identity, Vector3d.Right, (Fixed64)8, Fixed64.Quarter, out _));
    }

    [Theory]
    [InlineData(-3, false)]
    [InlineData(3, false)]
    [InlineData(-3, true)]
    [InlineData(3, true)]
    public void TryGetCenteredCapsuleContact_MatchesParallelCoreAtBothDomainLimits(int x, bool maximum)
    {
        Vector3d origin = new(Fixed64.Zero, Fixed64.Zero, maximum ? Fixed64.MaxValue : Fixed64.MinValue);
        Vector3d center = new((Fixed64)x, Fixed64.Zero, origin.Z);
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            origin, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            center, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)8, Fixed64.Quarter,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.One + Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Assert.Equal(Vector3d.Zero, Vector3d.Cross(offset, contact.Normal));
        Assert.True(!contact.FirstAnchor.TryGetPoint(out _) || !contact.SecondAnchor.TryGetPoint(out _));
    }

    [Theory]
    [InlineData(-3, false)]
    [InlineData(3, false)]
    [InlineData(-3, true)]
    [InlineData(3, true)]
    public void TryGetCenteredCapsuleContact_MatchesParallelCoreInAnObliqueTranslatedFrame(int x, bool nearLimit)
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver6);
        Vector3d origin = new((Fixed64)100, (Fixed64)(-20), nearLimit ? Fixed64.MaxValue - (Fixed64)8 : (Fixed64)7);
        Vector3d center = origin + rotation.Rotate(Vector3d.Right * (Fixed64)x);
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            origin, rotation, CubePoints, CubeTriangles, CubeEdges,
            center, rotation, Vector3d.Right, (Fixed64)8, Fixed64.Quarter,
            out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Vector3d tangent = Vector3d.Cross(offset, contact.Normal);
        // Allow final witness/normal rounding for this length-eight fixture;
        // classification and feature admission stay exact.
        Assert.True(FixedMath.Abs(tangent.X) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Y) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Z) <= Fixed64.FromRaw(16));
        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            origin, rotation, CubePoints, CubeTriangles, CubeEdges,
            center + contact.Normal * (contact.Depth + Fixed64.FromRaw(16)),
            rotation, Vector3d.Right, (Fixed64)8, Fixed64.Quarter, out _));
    }

    [Theory]
    [InlineData(false, false, 1)]
    [InlineData(true, false, 1)]
    [InlineData(true, true, 1)]
    [InlineData(false, false, -1)]
    [InlineData(true, false, -1)]
    public void ThinSupportFace_ShouldMatchObliqueCoreForBothBoxAndHull(bool hull, bool disjointTriangle, int side)
    {
        Fixed64 thickness = Fixed64.FromFraction(1, 100);
        Vector3d[] points = new Vector3d[CubePoints.Length];
        for (int i = 0; i < points.Length; i++)
            points[i] = new Vector3d(CubePoints[i].X, CubePoints[i].Y, CubePoints[i].Z * thickness);
        Vector3d axis = disjointTriangle ? Vector3d.Right : new Vector3d(1, -6, 0).Normalized;
        Vector3d center = disjointTriangle ? new Vector3d(Fixed64.FromFraction(29, 10), Fixed64.FromFraction(3, 4), Fixed64.Zero)
            : new Vector3d(Fixed64.FromFraction(3, 2), -Fixed64.FromFraction(21, 10), Fixed64.Zero);
        center *= (Fixed64)side;
        Fixed64 length = disjointTriangle ? (Fixed64)4 : (Fixed64)8;
        FixedContactAnchors contact;
        bool hit = hull ? FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, points, CubeTriangles, CubeEdges,
            center, FixedQuaternion.Identity, axis, length, Fixed64.Quarter, out contact)
            : new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity,
                new Vector3d(Fixed64.One, Fixed64.One, thickness)).TryGetCenteredCapsuleContact(
                    center, FixedQuaternion.Identity, axis, length, Fixed64.Quarter, out contact);
        Assert.True(hit);
        Assert.Equal(thickness + Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Vector3d tangent = Vector3d.Cross(offset, contact.Normal);
        Assert.True(FixedMath.Abs(tangent.X) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Y) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Z) <= Fixed64.FromRaw(16));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.True(FixedMath.Abs(point.X) <= Fixed64.One && FixedMath.Abs(point.Y) <= Fixed64.One);
    }

    [Theory]
    [InlineData(false, false, 1)]
    [InlineData(true, false, 1)]
    [InlineData(false, true, 1)]
    [InlineData(true, true, 1)]
    [InlineData(false, false, -1)]
    [InlineData(true, false, -1)]
    [InlineData(false, true, -1)]
    [InlineData(true, true, -1)]
    public void ObliqueCore_ShouldMatchACornerSupportForBothBoxAndHull(bool hull, bool asymmetric, int side)
    {
        Vector3d center = asymmetric ? new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Two, Fixed64.FromFraction(5, 4))
            : Vector3d.One * Fixed64.FromFraction(5, 4);
        center *= (Fixed64)side;
        Vector3d axis = (asymmetric ? new Vector3d(1, -2, 0) : new Vector3d(1, -1, 0)).Normalized;
        Fixed64 length = asymmetric ? (Fixed64)4 : Fixed64.Two;
        Fixed64 radius = asymmetric ? Fixed64.One : Fixed64.Half;
        FixedContactAnchors contact;
        Assert.True(hull ? FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            center, FixedQuaternion.Identity, axis, length, radius, out contact)
            : new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One).TryGetCenteredCapsuleContact(
                center, FixedQuaternion.Identity, axis, length, radius, out contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Vector3d.One * (Fixed64)side, point);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Vector3d tangent = Vector3d.Cross(offset, contact.Normal);
        Assert.True(FixedMath.Abs(tangent.X) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Y) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Z) <= Fixed64.FromRaw(16));
        Vector3d moved = center + contact.Normal * (contact.Depth + Fixed64.FromRaw(64));
        Assert.False(hull ? FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            moved, FixedQuaternion.Identity, axis, length, radius, out _)
            : new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One).TryGetCenteredCapsuleContact(
                moved, FixedQuaternion.Identity, axis, length, radius, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueCore_ShouldMatchTheSupportEdgeBeforeItsVertices(bool hull)
    {
        Vector3d center = new(Fixed64.FromFraction(3, 2), -Fixed64.FromFraction(21, 10), Fixed64.FromFraction(3, 2));
        Vector3d axis = new Vector3d(1, -6, 1).Normalized;
        FixedContactAnchors contact;
        Assert.True(hull ? FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            center, FixedQuaternion.Identity, axis, (Fixed64)8, Fixed64.Quarter, out contact)
            : new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One).TryGetCenteredCapsuleContact(
                center, FixedQuaternion.Identity, axis, (Fixed64)8, Fixed64.Quarter, out contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Fixed64.One, point.Y);
        // X/Z symmetry gives two equal-depth edges. Each representation keeps
        // its authored axis tie; both must retain the chosen edge interior.
        if (contact.Normal.Z == Fixed64.Zero)
        {
            Assert.Equal(Fixed64.One, point.X);
            Assert.True(FixedMath.Abs(point.Z) < Fixed64.One);
        }
        else
        {
            Assert.Equal(Fixed64.Zero, contact.Normal.X);
            Assert.Equal(Fixed64.One, point.Z);
            Assert.True(FixedMath.Abs(point.X) < Fixed64.One);
        }
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Vector3d tangent = Vector3d.Cross(offset, contact.Normal);
        Assert.True(FixedMath.Abs(tangent.X) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Y) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Z) <= Fixed64.FromRaw(16));
        Vector3d moved = center + contact.Normal * (contact.Depth + Fixed64.FromRaw(16));
        Assert.False(hull ? FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            moved, FixedQuaternion.Identity, axis, (Fixed64)8, Fixed64.Quarter, out _)
            : new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One).TryGetCenteredCapsuleContact(
                moved, FixedQuaternion.Identity, axis, (Fixed64)8, Fixed64.Quarter, out _));
    }

    [Fact]
    public void SideContact_ShouldRetainTheExactCoreEndpointOnTheSupportBoundary()
    {
        // Put Y faces first so the exact equal-depth face tie selects the side.
        int[] faces = new int[CubeTriangles.Length];
        CubeTriangles.AsSpan(CubeTriangles.Length - 12).CopyTo(faces);
        CubeTriangles.AsSpan(0, CubeTriangles.Length - 12).CopyTo(faces.AsSpan(12));
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, faces, CubeEdges,
            new Vector3d(3, 1, 0), FixedQuaternion.Identity, Vector3d.Right,
            (Fixed64)4, Fixed64.Quarter, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d capsule));
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.One - Fixed64.Quarter, Fixed64.Zero), capsule);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Assert.Equal(Vector3d.Zero, Vector3d.Cross(offset, contact.Normal));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleSupport_ShouldIgnoreRedundantDegenerateAuthoredTriangles(bool crossing)
    {
        int[] faces = new int[CubeTriangles.Length + 3 * CubePoints.Length];
        for (int vertex = 0; vertex < CubePoints.Length; vertex++)
            faces[3 * vertex] = faces[3 * vertex + 1] = faces[3 * vertex + 2] = vertex;
        CubeTriangles.CopyTo(faces, 3 * CubePoints.Length);
        Vector3d center = Vector3d.Right * (crossing ? (Fixed64)3 : Fixed64.FromFraction(3, 2));
        Vector3d axis = crossing ? Vector3d.Right : Vector3d.Up;
        Fixed64 length = crossing ? (Fixed64)8 : Fixed64.Zero;
        Fixed64 radius = crossing ? Fixed64.Quarter : Fixed64.One;
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, CubePoints, faces, CubeEdges,
            center, FixedQuaternion.Identity, axis, length, radius, out FixedContactAnchors contact));
        Assert.Equal(crossing ? Fixed64.One + Fixed64.Quarter : Fixed64.Half, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Assert.Equal(Vector3d.Zero, Vector3d.Cross(offset, contact.Normal));
    }

    [Fact]
    public void RotatedBoxAndHullCapsule_ShouldPairTheirFiniteEdgeWitnesses()
    {
        var box = new FixedOrientedBox(Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)7, (Fixed64)19, (Fixed64)(-11)),
            new Vector3d(2, 1, 3));
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4);
        Vector3d center = new(Fixed64.FromFraction(5, 4), Fixed64.Zero, Fixed64.Zero);
        Vector3d[] points = new Vector3d[CubePoints.Length];
        for (int index = 0; index < points.Length; index++)
            points[index] = new Vector3d(CubePoints[index].X * (Fixed64)2,
                CubePoints[index].Y, CubePoints[index].Z * (Fixed64)3);
        Assert.True(box.TryGetCenteredCapsuleContact(center, rotation, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, out FixedContactAnchors boxContact));
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(Vector3d.Zero,
            box.Orientation, points, CubeTriangles, CubeEdges, center, rotation,
            Vector3d.Up, Fixed64.Two, Fixed64.Half, out FixedContactAnchors hullContact));
        Assert.Equal(boxContact.Depth, hullContact.Depth);
        Assert.Equal(boxContact.Normal, hullContact.Normal);
        foreach (FixedContactAnchors contact in new[] { boxContact, hullContact })
        {
            Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
            Vector3d tangent = Vector3d.Cross(offset, contact.Normal);
            Assert.True(FixedMath.Abs(tangent.X) <= Fixed64.FromRaw(16));
            Assert.True(FixedMath.Abs(tangent.Y) <= Fixed64.FromRaw(16));
            Assert.True(FixedMath.Abs(tangent.Z) <= Fixed64.FromRaw(16));
            // Final rounded normal/depth need a few raw units to clear this oblique edge.
            Vector3d moved = center + contact.Normal * (contact.Depth + Fixed64.FromRaw(64));
            Assert.False(box.TryGetCenteredCapsuleContact(moved, rotation, Vector3d.Up,
                Fixed64.Two, Fixed64.Half, out _));
            Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(Vector3d.Zero,
                box.Orientation, points, CubeTriangles, CubeEdges, moved, rotation,
                Vector3d.Up, Fixed64.Two, Fixed64.Half, out _));
        }
    }

    [Fact]
    public void ZeroCoreCapsule_ShouldKeepTheSharedEndpointOfItsSupportEdge()
    {
        Vector3d center = new(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 2), -Fixed64.One);
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(Vector3d.Zero,
            FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            center, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero,
            Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(1, 1, -1), contact.FirstAnchor.LocalPoint);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d offset));
        Vector3d tangent = Vector3d.Cross(offset, contact.Normal);
        Assert.True(FixedMath.Abs(tangent.X) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Y) <= Fixed64.FromRaw(16));
        Assert.True(FixedMath.Abs(tangent.Z) <= Fixed64.FromRaw(16));
        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(Vector3d.Zero,
            FixedQuaternion.Identity, CubePoints, CubeTriangles, CubeEdges,
            center + contact.Normal * (contact.Depth + Fixed64.FromRaw(16)),
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.One, out _));
    }

    private static bool TryGetExtremeCapsuleContact(
        out FixedContactAnchors contact)
    {
        return FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            new Vector3d(
                Fixed64.MaxValue - (Fixed64)2,
                Fixed64.MinValue + (Fixed64)2,
                Fixed64.MaxValue - (Fixed64)2),
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MinValue + (Fixed64)2,
                Fixed64.MaxValue - (Fixed64)2),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out contact);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void PolluteStack(ulong seed)
    {
        Span<ulong> words = stackalloc ulong[2_048];
        for (int index = 0; index < words.Length; index++)
        {
            words[index] =
                seed
                + unchecked((ulong)index * 0x9E37_79B9UL);
        }
        GC.KeepAlive(words[unchecked((int)seed) & 2_047]);
    }
}
