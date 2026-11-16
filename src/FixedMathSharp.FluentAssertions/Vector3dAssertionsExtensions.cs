//=======================================================================
// Vector3dAssertionsExtensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Assertions;

/// <summary>Provides a FluentAssertions entry point for <see cref="Vector3d"/> values.</summary>
public static class Vector3dAssertionsExtensions
{
    /// <summary>Begins a fluent assertion about the supplied value.</summary>
    /// <param name="actualValue">The subject to capture by value.</param>
    /// <returns>An assertion object bound to the supplied value.</returns>
    public static Vector3dAssertions Should(this Vector3d actualValue)
    {
        return new Vector3dAssertions(actualValue);
    }
}
