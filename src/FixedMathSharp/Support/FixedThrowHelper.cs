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

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void ThrowIfArgument([DoesNotReturnIf(true)] bool condition, string? message = null)
    {
        if (condition)
            throw new ArgumentException(message);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void ThrowIfInvalid([DoesNotReturnIf(true)] bool condition, string? message = null)
    {
        if (condition)
            throw new InvalidOperationException(message);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void ThrowIfOutOfRange([DoesNotReturnIf(true)] bool condition, string? paramName = null, string? message = null)
    {
        if (condition)
            throw new ArgumentOutOfRangeException(paramName, message);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ThrowIfDivideByZero([DoesNotReturnIf(true)] bool condition, string? message = null)
    {
        if (condition)
            throw new DivideByZeroException(message ?? "Attempted to divide by zero.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ThrowIfArithmeticError([DoesNotReturnIf(true)] bool condition, string? message = null)
    {
        if (condition)
            throw new ArithmeticException(message ?? "An arithmetic error occurred.");
    }
}
