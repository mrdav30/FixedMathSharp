# Diagnostics Formatting

FixedMathSharp exposes human-readable formatting for diagnostics, logging,
editor display, and debugging. This surface is intentionally separate from
deterministic runtime math, raw Q32.32 representation, and serialization.

## Formatting Is Not Serialization

Use formatting when a person needs to read a value:

```csharp
Fixed64 speed = Fixed64.FromDouble(12.345);
string text = speed.ToString("0.00", CultureInfo.InvariantCulture); // "12.35"
```

Use serialization when data needs to roundtrip through storage or transport.
MemoryPack and JSON support preserve structured values; diagnostic strings are
not the serialization contract.

## Diagnostics APIs

The formatting surface follows familiar .NET shapes:

- `ToString()` returns an invariant diagnostic string.
- `ToString(string? format, IFormatProvider? provider)` supports standard
  numeric format strings where a type has numeric components.
- `TryFormat(Span<char> destination, out int charsWritten,
  ReadOnlySpan<char> format, IFormatProvider? provider)` writes into a caller
  supplied buffer and avoids the string allocation when the buffer is large
  enough.
- Runtime targets that support `ISpanFormattable`, such as `net8.0`, implement
  the interface. `netstandard2.1` exposes the same `TryFormat` method shape but
  cannot implement `ISpanFormattable` because the interface is unavailable.

Use `TryFormat` for repeated diagnostics, editor polling, or logging loops
where allocation pressure matters. Use `ToString` for low-frequency debugging
or convenience.

## Raw Values

Raw Q32.32 payloads are explicit:

```csharp
Fixed64 one = Fixed64.One;
string raw = one.ToRawString();            // "4294967296"
Fixed64 roundtrip = Fixed64.ParseRaw(raw); // Fixed64.One
```

Use raw APIs only when the text represents the already-scaled fixed-point
payload. For normal human-readable numbers, use `Parse` and `TryParse`:

```csharp
Fixed64 value = Fixed64.Parse("1.25", CultureInfo.InvariantCulture);
```

`Parse` and `TryParse` read value-space decimal text, reject non-finite or
invalid text, and treat values outside the representable Q32.32 range as
overflow. Use `FromDecimal` when the source value is already a `decimal` and
you want the same deterministic midpoint-to-even conversion path without
rounding through `double`.

This distinction matters because `"1.25"` and `"4294967296"` are different
representations of different concepts:

- `"1.25"` is a decimal diagnostic/value-space representation.
- `"4294967296"` is the raw payload for exactly `Fixed64.One`.

## Numeric Conversion

Formatting APIs produce text. They should not be used as a numeric conversion
path. Use explicit casts when a primitive value is needed at an engine or UI
boundary:

```csharp
double asDouble = (double)value;
float asFloat = (float)value;
```

Those conversions may lose precision because primitive floating-point types do
not have the same representation as Q32.32 fixed point. Keep deterministic
simulation state in FixedMathSharp values and convert only at boundaries.

## Exact Decimal Text

The current diagnostic formatter uses the same familiar .NET numeric formatting
behavior as `double` for human-readable display. That is convenient and
compatible with standard format strings, but it is not an exact fixed-point
decimal serializer.

If exact decimal text ever becomes a runtime requirement, it should be designed
as a dedicated deterministic integer-based formatter rather than layered onto
diagnostic formatting.
