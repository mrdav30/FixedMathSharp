# Full-Domain Arithmetic

`Fixed64` has a deliberately compact Q32.32 range. A complete expression can
still have a valid `Fixed64` answer even when a product, difference, or squared
distance in the middle is much larger.

FixedMathSharp's internal Wide layer keeps those intermediates exact until the
public API needs one final result. You continue to work with `Fixed64`, vectors,
transforms, and geometry; the wide representation stays internal.

## Why ordinary operators are sometimes not enough

Every overloaded C# operator finishes before the next operator begins. It
rounds and, when required by the operator contract, saturates its own result.

```csharp
Fixed64 value = new Fixed64(65_536);

Fixed64 chained = (value * value) / value;

bool succeeded = Fixed64.TryMultiplyDivide(
    value,
    value,
    value,
    out Fixed64 fused);
```

The multiplication in `chained` saturates before division can cancel the
growth. The fused call retains the complete ratio: `succeeded` is `true` and
`fused == value`.

Wide arithmetic also prevents premature rounding. A small product may round to
zero as a standalone `Fixed64` even when a later division would restore a
representable raw unit.

## Pick the contract you mean

| API shape | What happens | Use it when... |
| --- | --- | --- |
| Ordinary operators | Each operator rounds and saturates independently | Every intermediate is a meaningful public value |
| Fused value-returning methods | The expression rounds once; only the final result saturates | A saturated final answer is acceptable |
| `Try*` methods | Report the member's documented query or output failure without returning a partial result | The caller must distinguish success from that failure |
| Strict constructors/properties | Throw when the required public value cannot be represented | Saturation would misdescribe the value |
| `*ClippedToDomain` geometry APIs | Clip a conceptual shape only at the public coordinate domain | You explicitly want the representable portion of a shape |

`Try` does not mean “never throws.” Invalid arguments and violated input
contracts—such as a required normalized axis or nonnegative radius—may still
throw. The [API reference](https://mrdav30.github.io/FixedMathSharp/) documents
the exact failure and exception behavior of each member.

## Fused arithmetic example

```csharp
bool representable = Fixed64.TryMultiplyAdd(
    Fixed64.MaxValue,
    Fixed64.Two,
    -Fixed64.MaxValue,
    out Fixed64 result);

// representable is true.
// The oversized product cancels before narrowing, so result is MaxValue.
```

Scalar APIs include fused add/subtract and multiply/divide families. Matching
vector, matrix, transform, and geometry methods apply the same principle to a
whole operation and fail atomically when a required final output cannot be
represented.

## What this changes for consumers

### Transforms and rotations

A transformed point may leave Q32.32 temporarily, then return after translation
or relative-frame cancellation. Full-domain transform methods keep products and
translation terms together until final materialization.

### Geometry

Geometry often needs a reliable comparison more than it needs a materialized
distance. Wide predicates can compare squared distances, discriminants,
projections, and candidate features before converting the selected witness to
`Fixed64`.

That lets the library:

- classify containment, separation, and degeneracy from exact signs;
- rank candidates before a public distance saturates;
- retain rational or rigid-frame points until a world point is requested;
- round a final witness once; and
- build conservative clipped bounds without underestimating the shape.

This helps both extreme coordinates and ordinary near-cancelling expressions.
It does not increase the range of stored `Fixed64` state.

## How the internal layer is bounded

The implementation uses signed fixed-width values composed of 64-bit limbs:

| Internal type | Width | Typical role |
| --- | ---: | --- |
| `Signed192` | 192 bits | Endpoint differences, products, and foundational geometry |
| `Signed320` | 320 bits | Multi-component dot, cross, and determinant expressions |
| `Signed576` | 576 bits | Finite-axis ratios and roots |
| `Signed704` | 704 bits | Expanded finite-axis and radical evaluation |
| `Signed832` | 832 bits | The widest fixed-degree conic comparisons currently required |

These are not user-selectable precision modes. Each algorithm uses a width
proven sufficient for its bounded expression. Fixed-size value types and stack
scratch storage avoid a runtime `BigInteger` dependency and keep hot paths
allocation-light.

The Wide layer owns representation mechanics—carry, borrow, sign extension,
division, roots, comparison, and round-half-to-even narrowing. Public owners
still decide whether an operation saturates, clips, throws, or reports failure.

## Why Wide stays internal

Keeping the layer internal preserves a small public numeric API and prevents
limb layouts from becoming serialization contracts. The widths can evolve as
new bounded expressions are proven without asking users to choose or persist a
wide number format.

Gravitas is the runtime assembly's sole external production friend. It may
compose policy-neutral arithmetic behind Gravitas-owned physics semantics, but
Wide types do not belong in Gravitas public APIs or host adapters.

## Practical guidance

- Use ordinary operators when every intermediate is intentionally a standalone
  `Fixed64` value.
- Use an existing fused method when the complete expression is the operation
  you mean.
- Prefer a `Try*` form when a documented output failure must be distinct from a
  valid saturated value.
- Use relative coordinates, chunking, or rebasing when final application state
  itself exceeds Q32.32.
- Do not approximate an extreme deterministic predicate with floating point.
  Check the API for an existing full-domain operation first.

For ordinary representation and operator behavior, continue with
[Fixed64 Representation](fixed64-representation.md). For the geometry built on
these guarantees, see [Bounds and Geometry](bounds-and-geometry.md).
