using System;
using System.Collections.Generic;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedTriangleCapsulePlaneRayCrossFrameTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PhysicalDepth_ShouldRespectMemberScaleForEveryNativeEventRepresentation(bool capsule)
    {
        var triangle = new FixedTriangle(new(-8, 0, -8), new(8, 0, -8), new(0, 0, 8));
        Vector3d center = new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero);
        FixedQuaternion rotation = capsule ? FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)31) : FixedQuaternion.Identity;
        Fixed64 length = capsule ? Fixed64.One : Fixed64.Zero;
        List<Snapshot>? reference = null;
        int exactTies = 0, separated = 0, originalDisagreements = 0;
        foreach (FixedQuaternion memberRotation in new[] { FixedQuaternion.Identity,
            new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)37, Fixed64.Zero) })
        {
            var frame = new CapsulePlaneRayFrame(triangle,
                new CapsuleFiniteSectionFrame(Vector3d.Zero, memberRotation, center, rotation, length, Fixed64.One));
            int orientation = frame.GetWorldNormal(memberRotation).Y.m_rawValue > 0 ? 1 : -1;
            var events = new List<Snapshot>();
            Assert.True(CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, orientation,
                (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current,
                    CapsulePlaneRayEvent descriptor, int direction, scoped in CapsulePlaneRaySelection candidate) =>
                    events.Add(new Snapshot(candidate, current))));
            reference ??= events;
            foreach (Snapshot first in reference)
            foreach (Snapshot second in events)
            {
                var a = first.Borrow(); var b = second.Borrow();
                int comparison = a.CompareCoplanarDepth(b, first.Frame, second.Frame);
                Assert.Equal(-comparison, b.CompareCoplanarDepth(a, second.Frame, first.Frame));
                if (SamePhysicalPoint(a, first.Frame, b, second.Frame))
                {
                    Assert.Equal(0, comparison);
                    exactTies++;
                    if (a.CompareDepth(b) != 0) originalDisagreements++;
                }
                else
                {
                    Assert.True(a.TryGetDepth(first.Frame, out Fixed64 da));
                    Assert.True(b.TryGetDepth(second.Frame, out Fixed64 db));
                    // Distinct rounded values certify the same strict order of
                    // exact depths; equal rounded values do not certify a tie.
                    if (da != db) { Assert.Equal(da.CompareTo(db), comparison); separated++; }
                }
            }
        }
        Assert.True(exactTies > 0);
        Assert.True(separated > 0);
        Assert.True(originalDisagreements > 0);
    }

    private static bool SamePhysicalPoint(scoped in CapsulePlaneRaySelection first, in CapsulePlaneRayFrame firstFrame,
        scoped in CapsulePlaneRaySelection second, in CapsulePlaneRayFrame secondFrame)
    {
        const int words = CapsulePlaneRaySelection.FieldWords + 3;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var firstD = ContactQuadratic.At(values, signs, 0, words);
        var secondD = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic.Scale(first.Denominator, Signed576.ExtendValue(Signed320.ExtendValue(firstFrame.Finite.ShapeFrame.Denominator)), firstD);
        ContactQuadratic.Scale(second.Denominator, Signed576.ExtendValue(Signed320.ExtendValue(secondFrame.Finite.ShapeFrame.Denominator)), secondD);
        return ContactQuadratic.CompareRatios(first.X, firstD, first.Root, second.X, secondD, second.Root) == 0
            && ContactQuadratic.CompareRatios(first.Y, firstD, first.Root, second.Y, secondD, second.Root) == 0
            && ContactQuadratic.CompareRatios(first.Z, firstD, first.Root, second.Z, secondD, second.Root) == 0;
    }

    private sealed class Snapshot
    {
        private readonly ulong[] _values;
        private readonly int[] _signs;
        private readonly sbyte[] _valueSigns;
        private readonly CapsulePlaneRaySelectionMetadata _metadata;
        internal readonly CapsulePlaneRayFrame Frame;
        internal Snapshot(scoped in CapsulePlaneRaySelection value, in CapsulePlaneRayFrame frame)
        {
            _values = value.Values.ToArray(); _signs = value.Signs.ToArray(); _valueSigns = value.ValueSigns.ToArray();
            _metadata = value.Metadata; Frame = frame;
        }
        internal CapsulePlaneRaySelection Borrow()
        {
            var result = new CapsulePlaneRaySelection(_values, _signs, _valueSigns);
            result.Restore(_metadata); return result;
        }
    }
}
