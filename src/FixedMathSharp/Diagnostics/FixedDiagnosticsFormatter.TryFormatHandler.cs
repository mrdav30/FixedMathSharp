//=======================================================================
// FixedDiagnosticsFormatter.TryFormatHandler.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Buffers;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>TryFormatHandler declaration for FixedDiagnosticsFormatter.</content>
internal static partial class FixedDiagnosticsFormatter
{
    internal delegate bool TryFormatHandler(Span<char> destination, out int charsWritten);
}
