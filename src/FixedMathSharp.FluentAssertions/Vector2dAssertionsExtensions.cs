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

public static class Vector2dAssertionsExtensions
{
    public static Vector2dAssertions Should(this Vector2d actualValue)
    {
        return new Vector2dAssertions(actualValue);
    }
}
