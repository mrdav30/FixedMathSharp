//=======================================================================
// Fixed3x3AssertionsExtensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

/// <summary>Provides a FluentAssertions entry point for <see cref="Fixed3x3"/> values.</summary>
public static class Fixed3x3AssertionsExtensions
{
    /// <summary>Begins a fluent assertion about the supplied value.</summary>
    /// <param name="actualValue">The subject to capture by value.</param>
    /// <returns>An assertion object bound to the supplied value.</returns>
    public static Fixed3x3Assertions Should(this Fixed3x3 actualValue)
    {
        return new Fixed3x3Assertions(actualValue);
    }
}
