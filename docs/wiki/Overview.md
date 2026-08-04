# Technical Overview

FixedMathSharp is organized around one premise: deterministic runtime math needs
an explicit numeric representation and equally explicit behavior at every layer
built on top of it.

This page maps those layers and their responsibilities. Detailed numeric and
geometry contracts live on the focused wiki pages linked below; exact public
signatures live in the
[API documentation](https://mrdav30.github.io/FixedMathSharp/api/FixedMathSharp.html).

## System Shape

```text
Fixed64 and FixedMath
  -> vectors, rotations, and matrices
  -> bounds, primitives, curves, ranges, and deterministic random streams
  -> serialization and companion-package integrations

Internal fixed-width wide arithmetic
  -> exact intermediates shared by scalar and geometry operations
```

The public layers expose representable Q32.32 values. Internal wide mechanics
allow an operation to retain exact differences, products, sums, comparisons, and
roots until the contract requires one final public conversion.

## Scalar Foundation

`Fixed64` stores a signed Q32.32 value in a 64-bit integer. `FixedMath` owns the
shared deterministic scalar algorithms used by the rest of the library.

That ownership split keeps representation and conversion rules on `Fixed64`
while trigonometry, interpolation, roots, powers, rounding, and related
algorithms have one canonical implementation in `FixedMath`.

Read [Fixed64 Representation](fixed64-representation.md) for the raw layout,
range, precision, conversions, and guarded overflow behavior.

## API Ownership

| Layer                   | Primary owners                                                                                  | Responsibility                                                                           |
| ----------------------- | ----------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| Scalar representation   | `Fixed64`                                                                                       | Q32.32 storage, parsing, formatting, conversions, constants, and operators               |
| Scalar algorithms       | `FixedMath`                                                                                     | Shared deterministic rounding, interpolation, trigonometry, powers, roots, and utilities |
| Linear algebra          | `Vector2d`, `Vector3d`, `Vector4d`, `FixedQuaternion`, `Fixed3x3`, `Fixed4x4`, `FixedTransform` | Vectors, rotations, matrices, and transforms without engine dependencies                 |
| Geometry                | Bounds, rays, planes, segments, triangles, oriented boxes, and anchors                          | Dimension-explicit computational geometry and exact relation results                     |
| Deterministic utilities | `FixedCurve`, `FixedRange`, `DeterministicRandom`                                               | Repeatable interpolation, ranges, and seeded streams                                     |
| Wide mechanics          | Internal fixed-width numeric types                                                              | Policy-neutral exact intermediates, comparison, rounding, and narrowing support          |

Factories and convention-heavy operations stay on their owning types. Extension
classes expose curated receiver-shaped conveniences and forward to canonical
implementations rather than creating alternate algorithms.

Read [Coordinate Conventions](coordinate-conventions.md) before adding an engine
or host adapter. Read [Bounds and Geometry](bounds-and-geometry.md) for the
shape model, relation semantics, anchors, and boundary rules.

## Full-Domain Arithmetic

Ordinary public values remain `Fixed64` and fixed-point vectors. Some correct
operations still require intermediates wider than either the input or output
type—for example, subtracting opposite domain endpoints or comparing squared
distances before saturation.

The internal wide layer owns those representation mechanics. It is not a second
public number system and does not expose raw wide storage. Public operations
define when results round, saturate, throw, clip, or report failure.

Read [Full-Domain Wide Arithmetic](full-domain-wide-arithmetic.md) for the
invariants and narrowing model.

## Determinism Boundaries

Determinism depends on more than avoiding `float` and `double`. FixedMathSharp
also keeps these behaviors stable and explicit:

- rounding and overflow policy
- equality, normalization, and tie-breaking
- collection and result ordering
- random seeds and stream derivation
- coordinate and transform conventions
- serialization member order and package shape

Diagnostic strings are a separate contract from round-trip representations;
[Diagnostics Formatting](diagnostics-formatting.md) describes that boundary.

## Packages And Serialization

| Package family                                | Purpose                                                                 |
| --------------------------------------------- | ----------------------------------------------------------------------- |
| `FixedMathSharp`                              | Core runtime with MemoryPack serialization support                      |
| `FixedMathSharp.Lean`                         | The same math API without the direct MemoryPack dependency              |
| `FixedMathSharp.Chronicler` and `.Lean`       | Deterministic record-hash extensions for replay and conformance tooling |
| `FixedMathSharp.FluentAssertions` and `.Lean` | Test assertions for fixed-point values and core numeric types           |

MemoryPack layout is explicit through ordered serialization metadata. Lean
builds exclude MemoryPack-specific source while preserving the intended math
surface. JSON remains useful for human-readable tooling and interoperability
rather than hot deterministic state transfer.

## Validation And Performance

The xUnit suite protects arithmetic, geometry, serialization, deterministic
random behavior, and regressions across standard and Lean configurations. CI
runs supported build variants on Windows and Linux.

BenchmarkDotNet cases provide evidence for hot-path changes. Performance work is
accepted only when it preserves correctness, determinism, API semantics, and
serialization compatibility.

- [Coverage Report](https://mrdav30.github.io/FixedMathSharp/coverage/)
- [Benchmark Guide](https://github.com/mrdav30/FixedMathSharp/blob/main/tests/FixedMathSharp.Benchmarks/README.md)
- [API Documentation](https://mrdav30.github.io/FixedMathSharp/api/FixedMathSharp.html)

## Where To Go Next

- Start with [Fixed64 Representation](fixed64-representation.md) for numeric
  behavior.
- Use [Full-Domain Wide Arithmetic](full-domain-wide-arithmetic.md) when an
  algorithm crosses ordinary intermediate range.
- Use [Coordinate Conventions](coordinate-conventions.md) at adapter boundaries.
- Use [Bounds and Geometry](bounds-and-geometry.md) for spatial contracts.
- Use [Diagnostics Formatting](diagnostics-formatting.md) for diagnostic text,
  raw payload text, and serialization boundaries.
