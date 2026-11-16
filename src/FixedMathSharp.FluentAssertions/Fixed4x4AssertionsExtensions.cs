//=======================================================================
// Fixed4x4AssertionsExtensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Assertions;

/// <summary>Provides a FluentAssertions entry point for <see cref="Fixed4x4"/> values.</summary>
public static class Fixed4x4AssertionsExtensions
{
    /// <summary>Begins a fluent assertion about the supplied value.</summary>
    /// <param name="actualValue">The subject to capture by value.</param>
    /// <returns>An assertion object bound to the supplied value.</returns>
    public static Fixed4x4Assertions Should(this Fixed4x4 actualValue)
    {
        return new Fixed4x4Assertions(actualValue);
    }
}
