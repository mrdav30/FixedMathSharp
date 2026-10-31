//=======================================================================
// FixedQuaternionAssertionsExtensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

public static class FixedQuaternionAssertionsExtensions
{
    public static FixedQuaternionAssertions Should(this FixedQuaternion actualValue)
    {
        return new FixedQuaternionAssertions(actualValue);
    }
}
