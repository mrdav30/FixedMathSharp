# Fixed64 Representation

`Fixed64` is the scalar foundation of FixedMathSharp. It stores a deterministic fixed-point value in a signed 64-bit raw integer using a Q32.32 layout.

## Q32.32 Layout

The library-wide shift amount is fixed at 32 bits:

```csharp
public const int SHIFT_AMOUNT_I = 32;
public const long ONE_L = 1L << SHIFT_AMOUNT_I;
```

That means one whole unit is stored as `4,294,967,296` raw units. Conceptually:

```text
value = rawValue / 2^32
rawValue = value * 2^32
```

The lower 32 bits provide the fractional resolution. The upper side of the signed 64-bit value provides the whole-number range through normal two's-complement signed integer behavior.

## Scaled Integers, Not Stored Fractions

`Fixed64` does not store a whole-number part and a fractional object separately. It stores one signed integer, and the library interprets that integer through the fixed Q32.32 scale.

For example, `0.5` is stored as raw `0x00000000_80000000`, or `1L << 31`. That value is an integer. It means one half only because `Fixed64` interprets raw values as units of `1 / 2^32`.

This is the useful mental model:

```text
0.5       -> raw 2,147,483,648
1.0       -> raw 4,294,967,296
1 raw bit -> 1 / 4,294,967,296
```

Addition and subtraction can operate directly on raw values because both operands already share the same scale. Multiplication and division need rescaling because multiplying two Q32.32 values produces an intermediate value with twice as many fractional bits, while division needs enough shifted numerator precision before the quotient is taken.

So the fractional part does not disappear during bit shifts. It was never a separate stored thing. The fraction is the interpretation of the scaled integer, much like inches, centimeters, frames, or ticks are interpretations of a count at a chosen measurement scale.

## Range and Resolution

With Q32.32:

- Smallest positive raw step: `1 / 2^32`, approximately `0.00000000023283064365`.
- `Fixed64.One`: raw `0x00000001_00000000`.
- `Fixed64.MinIncrement`: raw `0x00000000_00000001`.
- `Fixed64.MIN_VALUE`: raw `long.MinValue`, exactly `-2147483648`.
- `Fixed64.MAX_VALUE`: raw `long.MaxValue`, just under `2147483648`.

Example raw values:

| Raw value | Meaning |
| --- | --- |
| `0x00000000_00000000` | `0` |
| `0x00000001_00000000` | `1` |
| `0x00000000_00000001` | `1 / 2^32` |
| `0x7FFFFFFF_FFFFFFFF` | Maximum representable positive value |
| `0x80000000_00000000` | Minimum representable negative value |

## Why This Trade-Off?

Q32.32 favors fine fractional precision while keeping a large enough whole-number range for many deterministic simulations, games, procedural systems, and tools. The trade-off is deliberate:

- More fractional bits reduce quantization error in small movements, rotations, interpolation, and accumulated simulation steps.
- Fewer whole-number bits than `double` or a wider integer-backed fixed type mean large worlds should usually use local coordinates, chunk-relative positions, rebasing, or domain-specific scaling.
- Arithmetic must guard overflow because multiply, divide, trigonometry, and interpolation can temporarily need more intermediate range than the final 64-bit value can store.

FixedMathSharp handles these cases with deterministic, guarded algorithms. For example, `Fixed64` multiplication uses full-width intermediate precision and saturating behavior for overflow paths.

## What About Other Shift Amounts?

A different fixed-point layout would choose a different range/precision balance. For example, a Q48.16 style layout would have much more whole-number range and much less fractional precision.

FixedMathSharp does not currently expose `SHIFT_AMOUNT_I` as a user-configurable setting. Changing it would affect constants, arithmetic algorithms, serialization compatibility, test expectations, and the practical behavior of every numeric type built on `Fixed64`.

Use Q32.32 when deterministic precision is more important than enormous coordinate range. If an application needs much larger absolute values, prefer changing the domain scale or coordinate system before changing the numeric representation.

## Practical Guidance

### Raw Longs Vs Integer Longs

A `long` can mean two different things around `Fixed64`: a whole-number value
that should be converted into Q32.32 space, or an already-scaled raw payload.
FixedMathSharp keeps those paths explicit instead of exposing `Fixed64 + long`
style arithmetic overloads that would have to guess.

Use an explicit conversion when the `long` is a normal whole-number value:

```csharp
long tileCount = 5;
Fixed64 distance = (Fixed64)tileCount;
```

Use `FromRaw` only when the `long` is already a Q32.32 raw value:

```csharp
long rawStep = 1;
Fixed64 smallestStep = Fixed64.FromRaw(rawStep);
```

The same distinction applies to text:

```csharp
Fixed64 value = Fixed64.Parse("1.25");                 // Decimal value text
Fixed64 preciseValue = Fixed64.FromDecimal(1.25m);     // Decimal value input
Fixed64 rawValue = Fixed64.ParseRaw("4294967296");     // Raw Q32.32 payload
string rawText = Fixed64.One.ToRawString();            // "4294967296"
```

The explicit `long` conversion saturates to `Fixed64.MinValue` or
`Fixed64.MaxValue` when the source integer is outside the representable Q32.32
whole-number range. This keeps overflow deterministic and makes raw-value
construction an intentional choice.

`Fixed64.Parse` and `Fixed64.FromDecimal` operate in normal value space and
throw `OverflowException` for decimal values outside the representable Q32.32
range. `Fixed64.TryParse` reports the same overflow as `false`.

- Use `Fixed64.FromRaw(long)` only when you intentionally want an exact raw representation.
- Use constructors, constants, and helpers such as `Fixed64.One`, `Fixed64.FromFraction`, and `FixedMath` methods for normal value-space code.
- Keep deterministic simulation state in fixed-point values, but convert to `float` or `double` at rendering and engine interop boundaries when needed.
- Treat overflow behavior as part of the numeric contract; avoid relying on primitive floating-point intuition for extreme values.

For human-readable formatting, see
[`diagnostics-formatting.md`](diagnostics-formatting.md). Diagnostic strings,
raw payload strings, JSON, and MemoryPack are separate representation concerns.
