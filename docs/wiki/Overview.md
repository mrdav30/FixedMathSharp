# Technical Overview

FixedMathSharp is built in layers. Each layer has one clear job, and all public
runtime values eventually reduce to the same Q32.32 `Fixed64` representation.

```text
Fixed64 + FixedMath
        |
        +-- vectors, quaternions, matrices, FixedTransform
        |
        +-- curves, ranges, and deterministic random streams
        |
        +-- bounds and computational geometry
```

An internal fixed-width Wide layer supports operations whose intermediate values
are larger than the final public result. It is an implementation tool, not a
second public number system.

## The scalar foundation

`Fixed64` owns representation-level behavior:

- Q32.32 storage and raw values
- constants and conversions
- parsing and formatting
- equality, comparison, and operators

`FixedMath` owns shared scalar algorithms such as rounding, interpolation,
trigonometry, roots, powers, and utility functions. Vector and geometry code use
those canonical implementations rather than maintaining alternate algorithms.

Read [Fixed64 Representation](fixed64-representation.md) for range, resolution,
conversion, rounding, and overflow details.

## Who owns what?

| Area                    | Main types                                                                    | Responsibility                                                 |
| ----------------------- | ----------------------------------------------------------------------------- | -------------------------------------------------------------- |
| Scalar values           | `Fixed64`                                                                     | Q32.32 representation, conversion, parsing, operators          |
| Scalar algorithms       | `FixedMath`                                                                   | Shared deterministic math                                      |
| Linear algebra          | `Vector2d`, `Vector3d`, `Vector4d`, `FixedQuaternion`, `Fixed3x3`, `Fixed4x4` | Vectors, rotations, matrices, and transforms                   |
| Transform hierarchy     | `FixedTransform`                                                              | Engine-neutral local components and derived hierarchy views    |
| Geometry                | Bounds, rays, planes, segments, triangles, oriented boxes, anchors            | Reusable dimension-explicit geometry                           |
| Deterministic utilities | `FixedCurve`, `FixedRange`, `DeterministicRandom`                             | Interpolation, ranges, and repeatable random streams           |
| Exact intermediates     | Internal Wide types                                                           | Products, differences, comparisons, roots, and final narrowing |

Factories and convention-heavy operations stay on the owning type. Extension
classes are curated conveniences that forward to those canonical APIs.

## What “deterministic” covers

Avoiding `float` and `double` inside the runtime is only the first step. The
library also makes these choices explicit and testable:

- midpoint rounding and overflow behavior
- normalization and equality rules
- stable tie-breaking and result ordering
- random seed and stream derivation
- coordinate and matrix conventions
- serialization member order and package shape

Diagnostic strings are intentionally separate from round-trip data. See
[Diagnostics Formatting](diagnostics-formatting.md) before using formatted text
outside logs, editors, or debugging tools.

## Full-domain operations

An ordinary overloaded operator finishes before the next operator begins. That
means an intermediate result can round or saturate even when the complete
expression would fit.

FixedMathSharp uses fused methods and internal Wide arithmetic when an operation
needs to preserve the complete expression through one final conversion. Public
APIs still return `Fixed64`, vectors, bounds, or explicit success/failure
results; callers never need to manage wide limbs.

Read [Full-Domain Arithmetic](full-domain-wide-arithmetic.md) for the difference
between ordinary operators, fused methods, `Try*` methods, and clipped geometry
factories.

## Packages

| Package family                              | Purpose                                          |
| ------------------------------------------- | ------------------------------------------------ |
| `FixedMathSharp`                            | Core math with MemoryPack support                |
| `FixedMathSharp.Lean`                       | Core math without a direct MemoryPack dependency |
| `FixedMathSharp.Chronicler` / `.Lean`       | Deterministic `ChronicleHashWriter` extensions   |
| `FixedMathSharp.FluentAssertions` / `.Lean` | Assertions for fixed-point tests                 |

Lean builds exclude the `*.MemoryPack.cs` partial files and replace the direct
MemoryPack dependency with `Chronicler.MemoryPackShim`. The intended public math
surface remains aligned with the standard package.

Engine integration is intentionally separate. Unity users should use
[FixedMathSharp-Unity](https://github.com/mrdav30/FixedMathSharp-Unity). Other
adapters should convert their host conventions at the boundary rather than
changing the core package.

## Validation and performance

The solution contains core, Chronicler, and FluentAssertions packages; core and
Chronicler xUnit projects; and a BenchmarkDotNet project. CI builds and tests
the complete solution in `Release` and `ReleaseLean` on Windows and Linux.

The published coverage report currently measures the core test project. The
benchmark suite is evidence for hot-path changes, not a substitute for
correctness tests.

- [Core test-suite coverage](https://mrdav30.github.io/FixedMathSharp/coverage/)
- [Benchmark guide](https://github.com/mrdav30/FixedMathSharp/blob/main/tests/FixedMathSharp.Benchmarks/README.md)
- [API reference](https://mrdav30.github.io/FixedMathSharp/)

## Continue reading

- [Coordinate Conventions](coordinate-conventions.md) for vectors, matrices,
  transforms, and adapters
- [Bounds and Geometry](bounds-and-geometry.md) for shape selection and query
  semantics
- [Getting Started](getting-started.md) for package setup and source builds
