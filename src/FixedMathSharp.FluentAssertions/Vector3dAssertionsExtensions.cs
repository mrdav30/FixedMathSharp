//=======================================================================
// Vector3dAssertionsExtensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FluentAssertions;
using FluentAssertions.Execution;
using FluentAssertions.Numeric;

namespace FixedMathSharp.Assertions;

public static class Vector3dAssertionsExtensions
{
    public static Vector3dAssertions Should(this Vector3d actualValue)
    {
        return new Vector3dAssertions(actualValue);
    }
}
