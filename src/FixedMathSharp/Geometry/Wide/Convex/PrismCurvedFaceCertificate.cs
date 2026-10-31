//=======================================================================
// PrismCurvedFaceCertificate.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>A finite full-support witness certified on the same oriented source plane.</summary>
/// <remarks>The producer certifies exact plane equality; payload rounding never ranks candidates.</remarks>
internal readonly struct PrismCurvedFaceCertificate
{
    internal readonly int Face;
    internal readonly FixedContactAnchors Contact;
    internal readonly bool Physical;
    internal PrismCurvedFaceCertificate(int face, FixedContactAnchors contact, bool physical)
    { Face = face; Contact = contact; Physical = physical; }
}
