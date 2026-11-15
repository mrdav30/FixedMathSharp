//=======================================================================
// Vector2dAssertionsExtensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

/// <summary>Provides a FluentAssertions entry point for <see cref="Vector2d"/> values.</summary>
public static class Vector2dAssertionsExtensions
{
    /// <summary>Begins a fluent assertion about the supplied value.</summary>
    /// <param name="actualValue">The subject to capture by value.</param>
    /// <returns>An assertion object bound to the supplied value.</returns>
    public static Vector2dAssertions Should(this Vector2d actualValue)
    {
        return new Vector2dAssertions(actualValue);
    }
}
