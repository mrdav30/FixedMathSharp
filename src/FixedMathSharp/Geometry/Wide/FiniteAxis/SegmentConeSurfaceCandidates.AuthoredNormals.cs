//=======================================================================
// SegmentConeSurfaceCandidates.AuthoredNormals.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Full-width authored normal parameters for exact normal-cone families.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    internal static bool TryGetAuthoredNormalContact(SegmentConeSurfaceCandidate candidate, WideAxis3 authoredNormal,
        out FixedContactAnchors contact)
    {
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        WideRigidProjection.TransformLocalAxis(geometry.Finite.ShapeFrame.Basis, authoredNormal,
            out Signed576 x, out Signed576 y, out Signed576 z);
        return candidate.Feature == ConeSurfaceFeature.Apex ? TryGetApexTouchContact(candidate, x, y, z, out contact)
            : candidate.Feature == ConeSurfaceFeature.Rim ? TryGetRimTouchContact(candidate, x, y, z, out contact)
            : TryGetAxisContact(candidate, x, y, z, out contact);
    }

    internal static bool TryGetAuthoredAxisParameterContact(SegmentConeSurfaceCandidate candidate, Fixed64 parameter,
        WideAxis3 authoredNormal, out FixedContactAnchors contact)
    {
        contact = default;
        if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.SegmentInterval || candidate.Input.Radius != Fixed64.Zero)
            return false;
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        if (!geometry.ContainsParameter(parameter))
            return false;
        WideRigidProjection.TransformLocalAxis(geometry.Finite.ShapeFrame.Basis, authoredNormal,
            out Signed576 x, out Signed576 y, out Signed576 z);
        return TryMaterializeAxis(geometry,
            Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(parameter))),
            Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(Fixed64.One))), x, y, z, out contact);
    }

    internal static bool TryGetAuthoredAxisEndpointContact(SegmentConeSurfaceCandidate candidate, bool upper,
        WideAxis3 authoredNormal, out FixedContactAnchors contact)
    {
        contact = default;
        if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.SegmentInterval || candidate.Input.Radius != Fixed64.Zero)
            return false;
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        WideAxis3 start = geometry.Finite.Transform(candidate.Input.Segment.Start);
        bool admitted = geometry.Finite.TryGetAxialInterval(start.Y, WideArithmetic.Negate(geometry.Edge.Y),
            out Signed320 lowerN, out Signed320 lowerD, out Signed320 upperN, out Signed320 upperD);
        System.Diagnostics.Debug.Assert(admitted);
        WideRigidProjection.TransformLocalAxis(geometry.Finite.ShapeFrame.Basis, authoredNormal,
            out Signed576 x, out Signed576 y, out Signed576 z);
        return TryMaterializeAxis(geometry, Signed576.ExtendValue(upper ? upperN : lowerN),
            Signed576.ExtendValue(upper ? upperD : lowerD), x, y, z, out contact);
    }

    internal static bool TryGetAuthoredNormalDotSign(SegmentConeSurfaceCandidate candidate,
        WideAxis3 authoredNormal, WideAxis3 authoredCovector, out int sign)
    {
        sign = 0;
        if (!TryGetAuthoredNormalContact(candidate, authoredNormal, out _))
            return false;
        sign = AuthoredNormalDotSign(authoredNormal, authoredCovector);
        return true;
    }

    internal static bool TryGetAuthoredAxisEndpointNormalDotSign(SegmentConeSurfaceCandidate candidate, bool upper,
        WideAxis3 authoredNormal, WideAxis3 authoredCovector, out int sign)
    {
        sign = 0;
        if (!TryGetAuthoredAxisEndpointContact(candidate, upper, authoredNormal, out _))
            return false;
        sign = AuthoredNormalDotSign(authoredNormal, authoredCovector);
        return true;
    }

    private static int AuthoredNormalDotSign(WideAxis3 normal, WideAxis3 covector) =>
        RationalNormalDotSign(Signed832.ExtendValue(Signed576.ExtendValue(normal.X)),
            Signed832.ExtendValue(Signed576.ExtendValue(normal.Y)), Signed832.ExtendValue(Signed576.ExtendValue(normal.Z)),
            Signed576.ExtendValue(covector.X), Signed576.ExtendValue(covector.Y), Signed576.ExtendValue(covector.Z));

    private static int NormalEdgeDot(Signed576 x, Signed576 y, Signed576 z, WideAxis3 edge)
    {
        // Authored normals have complete Signed320 inputs. Their relative
        // rational transform is <452 bits; segment directions are <198 bits.
        // The complete signed projection stays <652 bits, within Signed832.
        return WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(x, edge.X),
            WideArithmetic.MultiplySigned576ToSigned832(y, edge.Y)),
            WideArithmetic.MultiplySigned576ToSigned832(z, edge.Z)).Sign;
    }
}
