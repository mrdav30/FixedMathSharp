# Full-Domain Arithmetic And Wide Intermediates

`Fixed64` deliberately exposes a compact Q32.32 value domain. Complex
expressions, however, can require more intermediate range than their final
answer. FixedMathSharp's internal Wide arithmetic layer preserves those
intermediates so a representable result is not lost to an earlier saturation or
rounding step.

This is a consumer-visible correctness guarantee, not a public arbitrary-
precision number system. Callers continue to work with `Fixed64`, vectors,
transforms, and geometry while the library carries the wider representation
internally.

## The Intermediate-Saturation Problem

Every ordinary C# operator completes before the next operator begins. A
`Fixed64` multiplication therefore rounds and, if necessary, saturates before a
following division can cancel that growth:

```csharp
Fixed64 value = new Fixed64(65_536);

Fixed64 chained = (value * value) / value;
bool succeeded = Fixed64.TryMultiplyDivide(
    value,
    value,
    value,
    out Fixed64 fused);

// chained is not value because value * value saturated first.
// succeeded is true and fused equals value.
```

Parentheses make the evaluation order explicit, but they cannot ask the C#
compiler to fuse independently overloaded operators. Use a fused API when the
mathematical expression, rather than each intermediate `Fixed64`, is the
contract.

Premature rounding can be just as significant as saturation. For example, a
small multiplication can round to zero even though a later division would
restore a representable raw unit. The Wide layer retains the complete numerator
and denominator until one final round-half-to-even conversion.

## The Full-Domain Contract

Full-domain operations follow three rules:

1. Input raw values are widened before an unsafe sum, difference, product,
   ratio, square root, or geometric predicate is evaluated.
2. Signs, ordering, roots, containment, and candidate selection are decided from
   exact intermediates rather than saturated public approximations.
3. A public `Fixed64`, vector, point, distance, or bound is materialized once,
   using the API's documented final-result policy.

The final mathematical result can still lie outside Q32.32. Wide arithmetic does
not change the public numeric range; it prevents a temporary value from
corrupting a result that does fit.

| Public surface                                              | Final behavior                                                                                                          | Use when                                                                |
| ----------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| Ordinary operators                                          | Each operator rounds and saturates independently                                                                        | Every intermediate value is meaningful and expected to fit              |
| Fused value-returning methods such as `Fixed64.MultiplyAdd` | The combined expression rounds once and only the final result saturates                                                 | A saturated final value is an acceptable contract                       |
| `Try*` arithmetic, vector, transform, and geometry methods  | Returns `false` atomically when the required final result is not representable or another documented precondition fails | The caller must distinguish success from saturation or invalid input    |
| Strict constructors and derived properties                  | Throws the documented exception when a required result cannot be represented                                            | Clipping or saturation would misdescribe the value                      |
| Explicit `*ClippedToDomain` geometry APIs                   | Clips only the final conceptual boundary to the Q32.32 domain                                                           | The desired result is specifically the representable portion of a shape |

## What Consumers Gain

### Fused Scalar And Vector Expressions

`Fixed64.TryAddSubtract`, `TrySubtractSums`, `MultiplyAdd`, `TryMultiplyAdd`,
and the `TryMultiplyDivide` overloads preserve their complete expression. Vector
helpers apply matching contracts component by component and fail atomically
instead of returning a partly valid vector.

```csharp
if (!Fixed64.TryMultiplyAdd(
        Fixed64.MaxValue,
        Fixed64.Two,
        -Fixed64.MaxValue,
        out Fixed64 result))
{
    // The final result was not representable.
}

// result is Fixed64.MaxValue; the oversized product cancels before narrowing.
```

Magnitude, distance, weighted-average, normalization, and linear-combination
helpers also avoid feeding saturated squared terms or sums back into later
decisions.

### Transforms And Rotations

Transforming a point can temporarily exceed Q32.32 even when translation or a
relative frame brings the final coordinate back into range. Vector, quaternion,
matrix, and `FixedTransform` `TryTransform*` APIs retain the scaled products and
translation terms until final materialization. The same principle supports
inverse transforms, composite local points, full-domain quaternion
normalization, and robust matrix construction and decomposition.

### Geometry Across The Scalar Domain

Geometry needs exact predicates even when a convenient public measurement would
saturate. The Wide layer lets bounds, segments, triangles, rays, finite axes,
cones, slabs, and oriented boxes:

- compare candidates before converting distances to `Fixed64`;
- classify sides, containment, separation, and degeneracy from exact signs;
- retain rational points and rigid-frame anchors until a world point is needed;
- solve roots and discriminants without narrowing their coefficients early;
- compute conservative bounds without underestimating an extreme shape; and
- round a final witness once instead of repeatedly deforming its construction.

The result is not merely support for unusually large worlds. Near-cancelling
expressions, tiny representable directions, extreme mass ratios, and ordinary
geometry translated near a domain boundary all benefit from the same contract.

## How It Works Internally

The numeric core uses signed, fixed-width two's-complement values composed of
64-bit limbs:

| Internal type |    Width | Primary role                                                                   |
| ------------- | -------: | ------------------------------------------------------------------------------ |
| `Signed192`   |  3 limbs | Endpoint differences, products, sums, and foundational exact geometry          |
| `Signed320`   |  5 limbs | Multi-component dot, cross, determinant, and product-difference expressions    |
| `Signed576`   |  9 limbs | Finite-axis products, ratios, square roots, and higher-order geometry          |
| `Signed704`   | 11 limbs | Expanded finite-axis and radical evaluations                                   |
| `Signed832`   | 13 limbs | Conic discriminants and the widest fixed-degree comparisons currently required |

These widths are not public precision modes. Each owner selects a width proven
large enough for its bounded expression. `WideArithmetic` centralizes limb
addition, subtraction, multiplication, magnitude handling, comparisons,
division, square roots, and final guard/sticky-bit rounding. Focused owners such
as Wide normalization, weighted-average, transform, and geometry helpers build
policy-neutral operations on that foundation.

Fixed-size value types and stack-allocated scratch spans avoid a `BigInteger`
runtime dependency and are designed to remain allocation-free on simulation hot
paths. Explicit integer carry, borrow, sign extension, and round-half-to-even
rules keep results deterministic across the supported .NET targets.

## Why The Wide Types Stay Internal

The layer solves known bounded expressions; it is not intended to become a
general-purpose arbitrary-precision API. Keeping it internal:

- preserves a small and approachable public numeric surface;
- lets implementation widths evolve with proven arithmetic requirements;
- prevents raw limb layouts from becoming serialization or compatibility
  contracts; and
- keeps final overflow behavior attached to meaningful public operations.

Gravitas is the runtime assembly's sole intentional non-test friend. It may
compose policy-neutral Wide arithmetic behind Gravitas-owned contact, mass,
impulse, friction, and continuous-collision semantics. Wide types must not leak
through Gravitas public APIs, and changes to internals consumed by Gravitas are
coordinated release events. This relationship does not extend to other LSF
libraries or host adapters.

## Practical Guidance

- Use ordinary operators when every intermediate is intentionally a standalone
  `Fixed64` result.
- Use an existing fused method when the complete expression is the operation you
  mean.
- Prefer a `Try*` form when an unrepresentable final result must not be confused
  with a valid saturated value.
- Use relative coordinates, chunking, or rebasing when final application state
  itself exceeds Q32.32; wider intermediates do not enlarge stored `Fixed64`
  values.
- Do not hand-roll public wide arithmetic or use floating point to approximate
  an extreme deterministic predicate. Check for an existing full-domain API
  first.

Member-level XML documentation remains the source of truth for individual
preconditions and final-result behavior. For the Q32.32 representation and
ordinary operator semantics, see
[`fixed64-representation.md`](fixed64-representation.md). For the geometry
contracts built on the Wide layer, see
[`bounds-and-geometry.md`](bounds-and-geometry.md).
