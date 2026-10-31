//=======================================================================
// Fixed4x4AssertionsExtensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

public static class Fixed4x4AssertionsExtensions
{
    public static Fixed4x4Assertions Should(this Fixed4x4 actualValue)
    {
        return new Fixed4x4Assertions(actualValue);
    }
}
