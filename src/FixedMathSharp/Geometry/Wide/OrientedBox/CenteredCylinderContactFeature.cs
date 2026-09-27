//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
namespace FixedMathSharp.Geometry;

/// <summary>Exact parallel box-face/cylinder-cap ownership for manifold clipping.</summary>
internal readonly struct CenteredCylinderContactFeature
{
    internal CenteredCylinderContactFeature(int boxFaceAxis, int boxFaceSign, int cylinderCapSign)
    {
        BoxFaceAxis = boxFaceAxis;
        BoxFaceSign = boxFaceSign;
        CylinderCapSign = cylinderCapSign;
        IsCapFace = true;
    }

    internal int BoxFaceAxis { get; }
    internal int BoxFaceSign { get; }
    internal int CylinderCapSign { get; }
    internal bool IsCapFace { get; }
}
