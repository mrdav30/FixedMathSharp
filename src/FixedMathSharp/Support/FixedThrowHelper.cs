using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Support;

internal static class FixedThrowHelper
{
    /// <summary>
    /// Throws an ArgumentNullException if the provided argument is null.
    /// </summary>
    /// <param name="argument">The argument to check for null.</param>
    /// <param name="paramName">The name of the parameter that caused the exception.</param>
    /// <param name="message">An optional message to include in the exception.</param>
    /// <exception cref="ArgumentNullException">Thrown when the argument is null.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ThrowIfNull([NotNull] object? argument, string? paramName = null, string? message = null)
    {
        if (argument is null)
            ThrowArgumentNullException(paramName, message);
    }

    [DoesNotReturn]
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void ThrowArgumentNullException(string? paramName, string? message = null) =>
        throw new ArgumentNullException(paramName, message);
}
