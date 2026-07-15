# FixedMathSharp Foundation Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:subagent-driven-development` (recommended) or
> `superpowers:executing-plans` to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make FixedMathSharp the first-class deterministic source of truth for
scalar arithmetic, vector and quaternion invariants, reusable segment geometry,
and explicit X/Z planar transforms, then delete the corresponding Gravitas
workarounds and duplicate math and re-achieve 100% reachable coverage before
release.

**Architecture:** FixedMathSharp first corrects `/` and `FastDiv` to share one
round-half-to-even division core. It keeps saturating operators while adding
result-producing exact `Try*` APIs, fused multiply-divide, and full-domain
projection arithmetic. Existing vector, quaternion, segment, and transform
types then gain only the missing shared behavior: `Vector2d.IsNormalized`,
scale-safe quaternion normalization, quaternion-log domain repair, multi-turn quaternion creation,
segment intersection/closest-pair queries, and explicit X/Z planar transform
components. Gravitas consumes those APIs, deletes its duplicates, and retains
only physics policy such as GJK scaling, solver thresholds, and contact-cache
compatibility.

**Tech Stack:** C# 11, Q32.32 `Fixed64`, FixedMathSharp `Vector2d`/`Vector3d`,
`FixedQuaternion`, `FixedSegment2d`, `FixedSegment`, and `FixedTransform`, xUnit
v3, BenchmarkDotNet, Gravitas 2D/3D/mixed collision, queries, constraints, and
CCD.

## Global Constraints

- Correctness and determinism precede maintainability and performance.
- Preserve the existing saturating behavior of `Fixed64` and vector operators.
- Correct `Fixed64` division midpoint behavior to round-half-to-even, matching
  multiplication, conversions, midpoint arithmetic, and the documented
  division contract. Do not preserve the current half-away-from-zero defect.
- Do not make ordinary operator chains context-sensitive. Parentheses around a
  multiplication do not retain a wide intermediate; fused arithmetic must be
  requested explicitly.
- Do not introduce floating-point arithmetic or target-specific `Int128`
  behavior; FixedMathSharp must continue targeting `netstandard2.1` and
  `net8.0` consistently.
- Do not add public `CanAdd`, `CanSubtract`, or combinatorial
  `CanSubtractThen*` APIs. Use result-producing `Try*` operations and compose
  them where an algorithm needs multiple terms.
- Do not duplicate raw overflow detection or 64-by-64 full-width
  multiplication in Gravitas.
- Keep `FixedMath.Acos` strict. Make quaternion normalization scale-safe, then
  enforce the normalized-component domain inside `QuaternionLog` before calling
  `Acos`.
- Use explicit X/Z names for planar transform APIs. Under the existing
  `(x, y) -> (x, elevation, y)` embedding, positive `Vector2d` rotation maps to
  negative three-dimensional Y yaw.
- Do not add configurable transform planes, a second transform type, or generic
  normal-compatibility policy without a concrete consumer.
- Reuse `FixedSegment2d` and `FixedSegment`; do not transplant Gravitas
  geometry helpers as new parallel utility classes.
- Do not use `Debug.Assert` as a runtime correctness guard.
- Use temporary local project references in the source, test, and benchmark
  projects of each downstream solution. Leave every project-reference change
  unstaged and uncommitted.
- Leave all implementation and documentation changes unstaged for owner review;
  the repository owner will commit accepted work.
- Preserve 100% reachable line, branch, and method coverage in FixedMathSharp
  and 100% line and branch coverage in Gravitas.

---

## Decisions Locked By This Plan

1. `Fixed64.operator /` and `FixedMath.FastDiv` use one shared
   round-half-to-even core. For every representable `x`, exact binary reciprocal
   identities such as `x / Fixed64.Two == x * Fixed64.Half` must hold. No such
   identity is promised when the reciprocal was already rounded, such as
   `Fixed64.One / 3`.
2. `PositiveFixed64RawHighLimit` will not remain in Gravitas. If the
   FixedMathSharp implementation still needs that boundary internally, derive
   it from `long.MaxValue >> FixedMath.SHIFT_AMOUNT_I`; do not copy the hex
   literal downstream.
3. Hex literals provide no runtime advantage over decimal literals. Use them
   only where their bit layout is the clearest representation, and accompany
   them with a decimal or invariant-based explanation. Prefer named constants,
   shifts, `uint.MaxValue`, and `long.MaxValue` derivations.
4. Saturating operators remain the convenient default. Separate `Try*` methods
   are required because an operator can return only the resulting value and
   cannot also report whether saturation occurred. The separate surface is a
   semantic requirement; avoiding the current calculate-and-reverse-check work
   is a secondary performance benefit.
5. `FixedVectorDifference` is deleted from Gravitas. Component-exact vector
   addition and subtraction belong to FixedMathSharp.
6. Full-width projection accumulation and ordering belong to FixedMathSharp.
   Gravitas owns GJK shift selection, safe-product thresholds, sweep admission,
   and conservative-advancement policy.
7. Invalid `ConvexShape` source/target kind use is an internal programming
   error, not an ordinary query miss. It must fail explicitly in every build.
8. `TryMultiplyDivide` is a distinct fused operation, not a correction to the
   existing operators. It returns `false` for a zero divisor or an
   unrepresentable final result, writes `default` on failure, and does not fail
   merely because an ordinary intermediate `Fixed64` product would saturate or
   underflow.
9. Do not add saturating, throwing, vector, or arbitrary-factor fused variants
   without a concrete package consumer. The two-factor and three-factor scalar
   overloads are the complete approved public surface.
10. `Vector3d.IsNormalized` and `Vector4d.IsNormalized` retain their current
    squared-magnitude contract. Add the missing `Vector2d.IsNormalized` parity
    API; do not conflate unit-length validation with solver-owned normal-cache
    compatibility.
11. `FixedQuaternion` reuses the proven scale-relative four-component
    magnitude/normalization policy. Once every nonzero finite quaternion is
    normalized correctly, `QuaternionLog` clamps normalized `W` to `[-1, 1]`
    before strict `Acos`. No duplicate `SafeQuaternionLog` API is added.
12. Quaternion angle constructors accept multi-turn angles. Their trigonometric
    dependencies already reduce arbitrary representable radians; degree/radian
    conversion uses the fused arithmetic core so conversion does not saturate
    before division. No second public angle-wrapper abstraction is added.
13. `FixedTransform` remains one three-dimensional host shell but stores its
    authored position, rotation, and scale components explicitly instead of
    repeatedly decomposing an internal matrix. The matrix constructor performs
    its documented decomposition once.
    It gains `PositionXZ`, `RotationXZRadians`, and `ScaleXZ` plus one
    `Vector2d` constructor overload. The position and scale setters preserve the
    existing Y component; the rotation setter establishes a pure Y-axis
    rotation and therefore replaces pitch and roll.
14. `RotationXZRadians` follows `Vector2d.Rotate`: zero faces `Vector2d.Right`
    and positive angles rotate toward `Vector2d.Forward`. The backing
    quaternion therefore uses the negated Y-axis angle under the established
    X/Z embedding.
15. `FixedSegment2d` owns full-domain point projection, point distance, unique
    finite-segment intersection, and closest segment-pair queries. Collinear
    overlap is not mislabeled as a unique intersection. `FixedSegment` owns the
    existing finite 3D closest-pair behavior with symmetric
    Q32.32-resolution-degenerate handling; the misleading
    `Vector3d.ClosestPointsOnTwoLines` surface is deleted rather than forwarded.
    This does not claim a new full-domain 3D solver or introduce an
    intersection-classification hierarchy.
16. `FixedTransform` remains a general math/host type and preserves signed or
    zero authored scale. Gravitas collider scale represents physical dimensions,
    not reflection: every scale component consumed by a 2D or 3D collider must
    be strictly positive. Standalone transform scale and compound-part scale
    are validated before shape math; Gravitas does not silently take absolute
    values or attempt mesh winding reflection in this release.

## Current Review Findings

- `Fixed64.operator /` has claimed round-half-to-even behavior since the initial
  repository commit, but it increments every guarded quotient with a low bit of
  one. Exact midpoint magnitudes therefore round away from zero. Raw inputs
  `1`, `5`, and `9` expose `x / Two != x * Half`; raw inputs `3` and `7` agree
  only because the away-from-zero result is also the even neighbor. The April
  2026 scalar hardening corrected multiplication from half-up to true
  half-to-even but left both division implementations unchanged.
- `FixedMath.FastDiv` duplicates the same division loop and rounding defect.
  Correcting only the public operator would leave normalization and geometry
  paths with different arithmetic semantics.
- `FixedVectorDifference` detects saturation by performing a vector operation
  and then an inverse operation. The policy is useful, but the implementation
  belongs at the scalar/vector arithmetic source of truth.
- `GjkSimplexScale` duplicates raw signed-overflow predicates already inherent
  in `Fixed64` operator implementation.
- `ConvexSupportProjection` duplicates FixedMathSharp's 64-by-64-to-128
  multiplication and exposes only a conditionally safe two-word signed sum.
  Two or three full-domain difference-products require an accumulator wider
  than signed 128 bits.
- `SelectThreeTermShift` under-bounds negative odd-raw expansion components.
  For `point.X = Fixed64.MaxValue`, target X bounds at `Fixed64.MinValue`, and
  `expansionRadius = Fixed64.MinIncrement`, the positive radius bound becomes
  zero after shift one while an actual negative expansion component remains
  negative one raw unit. The selector approves a working shift whose third
  subtraction still saturates.
- The unrestricted `int shift` helpers silently treat values outside `0..2`
  inconsistently. Every public-internal entry must reject invalid values before
  shifting.
- Three `Debug.Assert` calls are the only guards against invalid release-state
  geometry behavior.
- CCD mobility currently rejects a representable response when
  `normalComponent = 1 / 65536`, `bodyInverseMass = 1`,
  `constrainedInverseMass = Fixed64.MinIncrement`, and `responseSpeed = 1`.
  The exact velocity delta is `65536`, but the ordinary inverse-mass ratio
  saturates before the small normal component reduces it.
- `Vector2d` is the only vector type without `IsNormalized`. Seeded normalized
  3D/4D probes remain well inside the existing squared-magnitude tolerance, so
  no broader predicate or tolerance overload is justified.
- Quaternion magnitude/normalization still sums saturating component squares.
  Extreme finite quaternions therefore fail to normalize even though the
  scale-relative vector path already solves the same problem. `QuaternionLog`
  can also retain `W = One + MinIncrement` after tolerance normalization and
  pass it to strict `Acos`; Gravitas duplicates the log solely to avoid that
  failure.
- `FromAxisAngle` and `FromEulerAngles` reject angles outside `[-Pi, Pi]` even
  though their fixed-point sine and cosine functions already reduce multi-turn
  inputs. Gravitas accumulates unbounded 2D rotation and can therefore throw
  while publishing an otherwise valid planar transform.
- `DegToRad` and `RadToDeg` multiply before dividing. Their final result can fit
  even when that ordinary product saturates, so removing quaternion angle
  guards without fused conversion would still produce incorrect extreme degree
  rotations. `AngleAxis` separately duplicates axis-angle construction and
  mishandles a zero axis.
- FixedMathSharp's positive 2D rotation sends `Right` toward `Forward`, while a
  positive Y quaternion sends embedded X/Z `Right` toward negative Z. Gravitas
  currently reads and writes its positive scalar 2D rotation as positive Y yaw,
  mirroring host presentation relative to 2D collision geometry.
- `FixedTransform` currently stores only a matrix, so every component getter and
  setter decomposes and rebuilds state. Rotation extraction cannot distinguish
  authored rotation from negative or zero scale, and the Unity-derived
  `LossyScale` name implies hierarchy behavior that `Parent` does not implement.
  Explicit component storage is both simpler and the only reliable source for
  the new planar rotation view.
- Once `FixedTransform.Scale` preserves sign, mechanically replacing
  `LossyScale` would feed negative dimensions into primitive bounds/radii while
  existing mesh validation rejects nonpositive scale. Gravitas needs one
  explicit positive collider-scale admission rule rather than shape-dependent
  accidental behavior.
- Gravitas `PlanarSegmentGeometry.ClosestPoint` and `DistanceSquared` duplicate
  existing `FixedSegment2d` behavior. Its `TryIntersect` rejects parallel and
  collinear segments, so its current name overstates the operation. Two more
  private 2D closest-segment-pair solvers duplicate the same endpoint-projection
  algorithm.
- `Vector3d.ClosestPointsOnTwoLines` actually solves finite segments despite
  its vector/line naming. Its zero-determinant branch can divide by the second
  segment's zero squared length, so a degenerate second segment is not handled
  symmetrically. `FixedSegment` is the existing ownership type for that
  operation.

---

### Task 1: FixedMathSharp Division Rounding Contract

**Status:** Complete on 2026-07-14 (`179debc`).

**Files:**

- Modify: `src/FixedMathSharp/Numerics/Scalars/Fixed64.Operators.cs`
- Modify: `src/FixedMathSharp/Core/FixedMath.cs`
- Modify: `docs/wiki/fixed64-representation.md`
- Test: `tests/FixedMathSharp.Tests/Numerics/Scalars/Fixed64.Tests.cs`
- Test: `tests/FixedMathSharp.Tests/Core/FixedMath.Tests.cs`
- Benchmark: `tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`

**Interfaces:**

- Produces no new public API. `Fixed64.operator /` and `FixedMath.FastDiv`
  preserve divide-by-zero and final saturation behavior while changing exact
  midpoint results from away-from-zero to round-half-to-even.
- Produces one internal unsigned-magnitude division core and one reusable final
  rounding helper:

```csharp
internal static Fixed64 DivideMagnitude(
    ulong dividendMagnitude,
    ulong divisorMagnitude,
    bool negative);

internal static ulong RoundGuardedQuotientToEven(
    ulong guardedQuotient,
    bool hasTrailingRemainder,
    out bool overflowed);
```

- `DivideMagnitude` requires a divisor magnitude in `1..2^63`, matching every
  signed raw `Fixed64` value while keeping the loop's remainder doubling within
  `ulong`. Its callers own the public divide-by-zero contract and determine the
  result sign before entering the unsigned core.
- `RoundGuardedQuotientToEven` treats bit zero as the guard bit. It increments
  only when that bit is set and either trailing remainder exists or the retained
  magnitude is odd.

- [x] **Step 1: Add the midpoint regression before changing source.**

```csharp
[Theory]
[InlineData(1L, 0L)]
[InlineData(3L, 2L)]
[InlineData(5L, 2L)]
[InlineData(-1L, 0L)]
[InlineData(-3L, -2L)]
[InlineData(-5L, -2L)]
public void DivideByTwo_MidpointsRoundToEven(long inputRaw, long expectedRaw)
{
    Fixed64 result = Fixed64.FromRaw(inputRaw) / Fixed64.Two;

    Assert.Equal(Fixed64.FromRaw(expectedRaw), result);
}
```

- [x] **Step 2: Add exact binary reciprocal identity tests** across
      `long.MinValue`, `long.MaxValue`, zero, and raw values `-9..9`:

```csharp
Assert.Equal(value * Fixed64.Half, value / Fixed64.Two);
Assert.Equal(value * Fixed64.Quarter, value / new Fixed64(4));
Assert.Equal(value * Fixed64.Eighth, value / new Fixed64(8));
```

- [x] **Step 3: Add below/at/above midpoint and fast-path tests.** For raw
      dividend `1`, use divisors `Fixed64.Two + Fixed64.MinIncrement`,
      `Fixed64.Two`, and `Fixed64.Two - Fixed64.MinIncrement`; require raw
      results `0`, `0`, and `1`. Repeat with a negative dividend and assert
      `FastDiv` exactly matches `/` for every positive divisor case.
- [x] **Step 4: Run the focused tests and confirm the exact-even midpoint cases
      fail while below/above midpoint behavior remains correct.**

```powershell
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter "FullyQualifiedName~Fixed64|FullyQualifiedName~FastDiv"
```

- [x] **Step 5: Extract the existing magnitude division loop** into
      `DivideMagnitude` and route both `/` and the positive-divisor `FastDiv`
      path through it. Keep `FastDiv`'s non-positive-divisor fallback and the
      public operator's divide-by-zero exception unchanged.
- [x] **Step 6: Replace the false banker-rounding branch** with
      `RoundGuardedQuotientToEven`. Preserve the loop's extra guard bit, pass
      whether its final remainder is nonzero as the sticky condition, and test
      rounded carry against the asymmetric positive `long.MaxValue` and
      negative `2^63` magnitude limits before constructing the result.
- [x] **Step 7: Add a deterministic `BigInteger` oracle in tests only.** Compare
      `/` and positive-divisor `FastDiv` against exact
      `abs(dividendRaw) * 2^32 / abs(divisorRaw)` quotient/remainder arithmetic.
      Cover seeded raw pairs plus exact ties, both signs, zero dividend,
      `long.MinValue`, `long.MaxValue`, divisor `long.MinValue`, final rounding
      carry, saturation, and divide by zero.
- [x] **Step 8: Document the arithmetic contract** in
      `fixed64-representation.md`: multiplication and division round exact
      midpoints to the even raw value; exact binary reciprocal identities hold;
      pre-rounded reciprocals such as one third can still differ from direct
      division.
- [x] **Step 9: Run the scalar and `FastDiv` tests in `Release` and
      `ReleaseLean`, then run exact FixedMathSharp coverage.** Require all
      guard, sticky, retained-parity, sign, carry, saturation, and zero-divisor
      branches to be covered.
- [x] **Step 10: Benchmark `Divide` and `FastDiv`** before and after the shared
      core. Require zero allocations and no material regression; optimize the
      shared loop rather than restoring duplicated rounding implementations.
- [x] **Step 11: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

**Result:**

- Replaced the duplicated false banker-rounding branches with one guarded
  unsigned division core shared by `/` and positive-divisor `FastDiv`. Exact
  midpoints now round to the even raw value; divide-by-zero, sign, saturation,
  and the non-positive `FastDiv` fallback remain intact.
- Added midpoint, exact binary reciprocal, below/at/above boundary, signed
  saturation, rounded-carry, and 2,048-pair seeded `BigInteger` oracle tests.
  The initial regression run produced the expected seven failures before the
  shared-core fix.
- Final verification passed 1,186 FixedMathSharp plus 7 Chronicler tests in
  `Release`, and 1,165 FixedMathSharp plus 7 Chronicler tests in `ReleaseLean`.
  `DivideMagnitude` and `RoundGuardedQuotientToEven` reached 100% line and
  branch coverage. Remaining repository-wide gaps are reserved for Task 11.
- Focused BenchmarkDotNet `ShortRun` evidence remained allocation-free:
  `Divide` moved from 5.319 us to 5.144 us and `FastDiv` from 4.800 us to
  4.724 us. Treat these short-run timings as regression checks, not canonical
  performance claims.
- Independent review found one overly broad internal divisor contract. Source,
  tests, and this plan now consistently constrain the divisor magnitude to
  `1..2^63`; the follow-up review returned no remaining findings.

---

### Task 2: FixedMathSharp Exact Add And Subtract Contract

**Files:**

- Modify: `src/FixedMathSharp/Numerics/Scalars/Fixed64.Operators.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector2d.Statics.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector3d.Statics.cs`
- Test: `tests/FixedMathSharp.Tests/Numerics/Scalars/Fixed64.Tests.cs`
- Test: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Test: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`
- Benchmark: `tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`
- Benchmark: `tests/FixedMathSharp.Benchmarks/Vector2dBenchmarks.cs`
- Benchmark: `tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs`

**Interfaces:**

- Produces:

```csharp
public partial struct Fixed64
{
    public static bool TryAdd(Fixed64 left, Fixed64 right, out Fixed64 result);
    public static bool TrySubtract(Fixed64 left, Fixed64 right, out Fixed64 result);
}

public partial struct Vector2d
{
    public static bool TryAdd(Vector2d left, Vector2d right, out Vector2d result);
    public static bool TrySubtract(Vector2d left, Vector2d right, out Vector2d result);
}

public partial struct Vector3d
{
    public static bool TryAdd(Vector3d left, Vector3d right, out Vector3d result);
    public static bool TrySubtract(Vector3d left, Vector3d right, out Vector3d result);
}
```

- On success, `result` is the exact Q32.32 result.
- On failure, `result` is `default`; no partially saturated vector is exposed.
- Existing operators preserve their current saturated result.

- [x] **Step 1: Add scalar red tests** covering ordinary values, exact
      `MinValue`/`MaxValue` boundaries, positive and negative overflow, default
      failure output, and unchanged saturating operator results.
- [x] **Step 2: Run the focused scalar tests and confirm the new API tests fail.**

```powershell
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter "FullyQualifiedName~Fixed64"
```

- [x] **Step 3: Extract one private raw add/subtract overflow core** and route
      both the existing operators and new public `Try*` methods through it.
      Mark the small hot methods for aggressive inlining; do not maintain two
      independent bit predicates.
- [x] **Step 4: Add component-atomic vector `Try*` methods** using the scalar
      methods. Set the vector result only after every component succeeds.
- [x] **Step 5: Add vector boundary tests** for a failing first, middle, and
      final component plus ordinary exact translations and differences.
- [x] **Step 6: Run the scalar and vector tests in `Release` and
      `ReleaseLean`.**
- [x] **Step 7: Benchmark scalar and vector `Try*` success paths** against the
      current calculate-and-inverse-check pattern. Require zero allocations and
      no material hot-path regression; optimize the shared raw core rather than
      adding a second API shape.
- [x] **Step 8: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

**Result:**

- Added exact, non-saturating `TryAdd` and `TrySubtract` APIs to `Fixed64`,
  `Vector2d`, and `Vector3d`. Exact `MinValue`/`MaxValue` results succeed;
  overflow returns `false` with `default`, and vector results remain atomic.
  Existing scalar and vector operators retain their saturating contract.
- Routed the operators and new scalar APIs through one aggressively-inlined
  signed-overflow predicate. The first boolean-selector form was rejected by
  the benchmark gate after it slowed vector operators; the final precomputed
  sign-mask form restored the hot paths without duplicating overflow logic.
- Added scalar positive/negative overflow and exact-boundary regressions plus
  2D and 3D first/middle/final component-failure coverage. The red runs failed
  with the expected missing-API compiler errors before implementation.
- Final verification passed 1,196 FixedMathSharp plus 7 Chronicler tests in
  `Release`, and 1,175 FixedMathSharp plus 7 Chronicler tests in `ReleaseLean`.
  The shared predicate and all six public `Try*` methods reached 100% line and
  branch coverage. Remaining repository-wide gaps are reserved for Task 11.
- Focused BenchmarkDotNet `ShortRun` evidence was allocation-free. Compared
  with calculate-and-inverse-check, exact `Try*` reduced successful scalar work
  from 1.053 us to 661.3 ns, `Vector2d` from 1.811 us to 1.056 us, and
  `Vector3d` from 2.712 us to 1.588 us. Existing operator benchmarks were flat
  or faster than their pre-change baselines; the largest observed regression
  was approximately 0.3%, within short-run noise.
- Independent review found no critical, important, or minor issues and judged
  the task ready for owner review.

---

### Task 3: FixedMathSharp Full-Domain Projection Arithmetic

**Files:**

- Modify: `src/FixedMathSharp/Numerics/Scalars/Fixed64.Operators.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector2d.Statics.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector3d.Statics.cs`
- Test: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Test: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`
- Benchmark: `tests/FixedMathSharp.Benchmarks/Vector2dBenchmarks.cs`
- Benchmark: `tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs`

**Interfaces:**

- Consumes: Task 2 arithmetic and the existing private full-width multiplication
  implementation.
- Produces:

```csharp
public partial struct Vector2d
{
    public static int CompareProjection(
        Vector2d candidate,
        Vector2d current,
        Vector2d direction);
}

public partial struct Vector3d
{
    public static int CompareProjection(
        Vector3d candidate,
        Vector3d current,
        Vector3d direction);

    public static Fixed64 ProjectNonNegativeDifference(
        Vector3d target,
        Vector3d source,
        Vector3d direction);
}
```

- `CompareProjection` returns the exact sign of
  `(candidate - current) dot direction` without intermediate saturation and
  without requiring a normalized direction.
- `ProjectNonNegativeDifference` accumulates before Q32.32 conversion, floors
  positive fractional remainder to preserve a conservative lower bound, clamps
  negative results to zero, and saturates only the final positive result.

- [x] **Step 1: Add red tests** for positive, negative, and cancelling
      projections across the complete component range, including a three-term
      positive sum and negative sum that overflow a signed 128-bit accumulator.
- [x] **Step 2: Add red conservative-projection tests** for zero, negative,
      one-unit, fractional-floor, and final-result saturation behavior.
- [x] **Step 3: Run the focused vector tests and confirm the new APIs fail.**
- [x] **Step 4: Implement the full-domain projection core** inside `Fixed64`.
      Represent `candidate - current` and `target - source` raw differences as
      sign plus an unsigned 65-bit magnitude instead of first constructing a
      saturated `Fixed64`. Multiply each 65-bit difference by the 64-bit
      direction component into a signed three-word product, accumulate with
      carry/sign extension, and decide sign/order from the complete sum. Keep
      the word types internal so vector callers share one implementation.
- [x] **Step 5: Convert the positive Q64.64 sum to Q32.32 only after checking
      the complete high words.** If a local high-word boundary is needed, derive
      it from `long.MaxValue >> FixedMath.SHIFT_AMOUNT_I` and document the
      invariant rather than its hexadecimal spelling.
- [x] **Step 6: Run the focused tests in `Release` and `ReleaseLean`.**
- [x] **Step 7: Benchmark ordinary support-projection comparisons and extreme
      inputs.** Require zero allocations and compare the same fixture before
      and after the wide-accumulator change.
- [x] **Step 8: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

**Result:**

- Added exact `Vector2d.CompareProjection`, `Vector3d.CompareProjection`, and
  conservative `Vector3d.ProjectNonNegativeDifference` APIs. One internal
  signed three-word accumulator now handles complete raw differences, extreme
  products, cancellation, flooring, and final-only saturation while reusing
  the existing allocation-free 64-by-64 multiplier.
- Added full-domain endpoint, signed-128-overflow, cancellation, carry,
  `MinValue` direction, zero/negative, one-unit, fractional-floor,
  sub-quantum, upper-representability-boundary, and final-saturation tests. The
  initial red run failed with the expected missing-API compiler errors; the
  upper boundary test was also mutation-checked against an incorrect `>=`
  comparison before restoring the correct `>` contract.
- Final verification passed 1,204 FixedMathSharp plus 7 Chronicler tests in
  `Release`, and 1,183 FixedMathSharp plus 7 Chronicler tests in `ReleaseLean`.
  All seven Task 3 methods reached 100% line and branch coverage; remaining
  repository-wide gaps are reserved for Task 11. A separate seeded
  `BigInteger` oracle matched 4,096 full-raw-domain 3D tuples.
- Focused BenchmarkDotNet `ShortRun` evidence was allocation-free. The exact
  ordinary comparison measured 2.202 us versus 2.608 us for saturating 2D and
  3.266 us versus 3.717 us for saturating 3D. On extreme fixtures, exact 2D
  measured 1.158 us versus 1.990 us and exact 3D measured 1.760 us versus
  3.013 us. Treat these short-run timings as regression checks, not canonical
  performance claims.
- Independent review found the missing upper representability-boundary
  regression. After adding it and the recommended sub-quantum floor case, the
  follow-up review returned no remaining findings and judged the task ready for
  owner review.

---

### Task 4: FixedMathSharp Fused Multiply-Divide

**Files:**

- Add: `src/FixedMathSharp/Numerics/Scalars/Fixed64.MultiplyDivide.cs`
- Test: `tests/FixedMathSharp.Tests/Numerics/Scalars/Fixed64.Tests.cs`
- Benchmark: `tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`

**Interfaces:**

- Consumes: Task 1's verified `RoundGuardedQuotientToEven` policy plus the
  existing `AbsToUInt64` and `Multiply64To128` arithmetic core.
- Produces:

```csharp
public partial struct Fixed64
{
    public static bool TryMultiplyDivide(
        Fixed64 left,
        Fixed64 right,
        Fixed64 divisor,
        out Fixed64 result);

    public static bool TryMultiplyDivide(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third,
        Fixed64 divisor,
        out Fixed64 result);
}
```

- For raw inputs `A`, `B`, `C`, and `D`, the two-factor overload rounds
  `(A * B) / D` to the nearest raw integer and the three-factor overload rounds
  `(A * B * C) / (D * 2^32)` to the nearest raw integer. Both use
  round-half-to-even exactly once.
- A zero divisor or final magnitude outside `long.MinValue..long.MaxValue`
  returns `false` with `result = default`. Exact `long.MinValue` remains a
  successful negative result.
- The public `Try*` overloads and package-internal conversion callers share one
  fused core that reports final representability and the correctly saturated
  result. This lets `FixedMath.DegToRad`/`RadToDeg` preserve their value-returning
  contracts without calculate-and-reverse checks or duplicate wide arithmetic.

- [x] **Step 1: Add two-factor red tests** using raw-value operands for all sign
      combinations, exact zero, exact `MinValue`/`MaxValue`, divisor zero,
      positive and negative final overflow, default failure output, and
      half-to-even cases `1 * 1 / 2 -> 0` and `3 * 1 / 2 -> 2` raw units.
- [x] **Step 2: Add the rescued-intermediate regressions.** Prove that
      `65536 * 65536 / 65536` returns `65536` although the ordinary product
      saturates, and that `MinIncrement * Half / Half` returns `MinIncrement`
      although the ordinary product rounds to zero.
- [x] **Step 3: Run the focused scalar tests and confirm both overloads are
      missing.**

```powershell
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter "FullyQualifiedName~Fixed64"
```

- [x] **Step 4: Implement the two-factor overload** with the existing unsigned
      64-by-64-to-128 multiplier and one allocation-free unsigned divide with
      quotient/remainder. Feed its guard and sticky state through Task 1's
      rounding helper, then apply the result sign after comparing the rounded
      magnitude against the asymmetric positive and negative limits.
      Keep the result-producing core internal so Task 5's angle conversions can
      reuse its final saturation result while public `TryMultiplyDivide` writes
      `default` on failure.
- [x] **Step 5: Add three-factor red tests** matching the two-factor sign,
      boundary, zero-divisor, default-output, and half-to-even coverage. Add the
      CCD regression with raw values `2^16`, `2^32`, `2^32`, and `1`; require
      the representable raw result `2^48` (`65536`).
- [x] **Step 6: Implement the three-factor overload** by extending the existing
      unsigned product to three 64-bit words. Divide that numerator by the
      64-bit divisor, retain the quotient's low 32 discarded bits plus the
      division remainder, reduce them to one guard bit plus sticky state, and
      use Task 1's rounding helper to produce the final Q32.32 raw result once.
      Do not use `BigInteger`, floating point, `Int128`, or a public wide numeric
      type in runtime code.
- [x] **Step 7: Add deterministic oracle coverage** in the test project using
      `BigInteger` only as an independently computed reference. Cover fixed
      boundary vectors plus a seeded set of raw inputs for both overloads,
      including cases where ordinary grouping saturates or underflows but the
      fused result fits.
- [x] **Step 8: Run the focused tests in `Release` and `ReleaseLean`, then run
      exact FixedMathSharp coverage.** Every divisor-zero, sign, carry,
      quotient-overflow, rounding, and `long.MinValue` branch must be reachable.
- [x] **Step 9: Benchmark both overloads** against their ordinary operator
      chains for common in-range inputs and rescued extreme inputs. Require zero
      allocations; optimize a proven common path without weakening the
      single-rounding contract.
- [x] **Step 10: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

**Result:**

- Added the complete approved two-factor and three-factor
  `Fixed64.TryMultiplyDivide` surface. The allocation-free core preserves the
  exact 128-bit or 192-bit unsigned numerator, divides before applying one
  final round-half-to-even operation, accepts exact `long.MinValue`, and
  rejects only a zero divisor or an unrepresentable final result. Public
  failure writes `default`; the shared internal core retains the correct signed
  saturation for Task 5's value-returning conversion callers.
- Reused the existing 64-by-64 multiplier and Task 1 rounding helper. A
  measured common path cancels shared powers of two before unsigned division;
  no runtime `BigInteger`, floating point, `Int128`, public wide-number type, or
  allocation was introduced. The cohesive implementation moved into
  `Fixed64.MultiplyDivide.cs` rather than pushing `Fixed64.Operators.cs` across
  the repository's source-size warning.
- Added sign, zero, exact-boundary, divisor-zero, positive/negative overflow,
  midpoint, saturated-intermediate, rounded-to-zero-intermediate, CCD, carry,
  maximum-remainder, 63-bit cancellation, and `long.MinValue` factor/divisor
  regressions. A test-only `BigInteger` oracle matched fixed boundary vectors,
  2,048 seeded full-raw-domain tuples per overload, and 768 magnitude-stratified
  three-factor tuples whose final results are explicitly required to remain
  representable. The two red stages failed with the expected missing-overload
  compiler errors before implementation.
- Final verification passed 1,244 FixedMathSharp plus 7 Chronicler tests in
  `Release`, and 1,223 FixedMathSharp plus 7 Chronicler tests in `ReleaseLean`.
  All ten Task 4 methods reached 100% line and branch coverage; remaining
  repository-wide gaps are reserved for Task 11.
- Focused BenchmarkDotNet `ShortRun` evidence covered 256 operations per row
  with zero allocations. Two-factor ordinary chain/fused means were
  6.003/6.493 us and rescued-extreme means were 4.946/5.310 us. Three-factor
  ordinary chain/fused means were 8.400/14.551 us and rescued-extreme means
  were 6.243/12.262 us. The three-factor cost is the measured price of retaining
  a full 192-bit product and one final rounding where the cheaper operator
  chain can saturate or discard information; treat these short-run timings as
  regression evidence rather than canonical performance claims.
- Independent review found no production-arithmetic defect after an additional
  boundary-grid and 100,000-sample exact-arithmetic comparison. It identified
  missing representable carry and `long.MinValue` divisor/third-factor oracle
  vectors; after those focused tests and magnitude-stratified samples were
  added, follow-up review returned no remaining findings and judged the task
  ready for owner review.

---

### Task 5: Vector And Quaternion Full-Domain Contracts

**Files:**

- Modify: `../FixedMathSharp/src/FixedMathSharp/Core/FixedMath.Trigonometry.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector2d.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Rotations/FixedQuaternion.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Rotations/FixedQuaternion.Statics.cs`
- Modify: `../FixedMathSharp/docs/wiki/coordinate-conventions.md`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Core/FixedTrigonometry.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Rotations/FixedQuaternion.Tests.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Vector2dBenchmarks.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/QuaternionBenchmarks.cs`

**Interfaces:**

- Consumes: Task 1's corrected division contract, Task 4's internal fused
  multiply-divide result core, and the existing scale-relative `Vector3d` and
  `Vector4d` magnitude/normalization pattern.
- Produces:

```csharp
public partial struct Vector2d
{
    public readonly bool IsNormalized();
}
```

- `FixedQuaternion.GetMagnitude` and `GetNormalized` keep their signatures but
  work for every nonzero finite component combination without saturating the
  sum of squares. Normalizing the zero quaternion continues to return
  `FixedQuaternion.Identity`.
- `FixedQuaternion.FromAxisAngle`, `FromEulerAngles`,
  `FromEulerAnglesInDegrees`, and `AngleAxis` keep their signatures but accept
  every representable angle. Radian constructors rely on deterministic
  trigonometric reduction; degree constructors first use full-domain fused
  conversion.
- `FixedMath.DegToRad` returns the correctly rounded finite conversion for every
  `Fixed64` input because its mathematical result always fits. `RadToDeg`
  preserves final saturation when the converted degree value is genuinely out
  of range, but no longer saturates merely because its product intermediate
  does not fit.
- `QuaternionLog` keeps its signature. It scale-safely normalizes every nonzero
  finite quaternion, clamps normalized `W` to `[-1, 1]`, and then calls strict
  `FixedMath.Acos`. No solver-only safe-log variant or gross-failure branch is
  introduced.

- [ ] **Step 1: Add `Vector2d.IsNormalized` red tests** for `UnitX`, `UnitY`,
      zero, length two, a normalized non-axis vector, and squared magnitudes one
      raw unit inside and outside the accepted epsilon boundary.
- [ ] **Step 2: Add normalized-result parity coverage** for deterministic tiny,
      ordinary, and extreme nonzero 2D/3D/4D inputs. Add quaternion cases with
      one and four `Fixed64.MaxValue` or `Fixed64.MinValue` components. Assert
      every nonzero normalized quaternion reports normalized, normalization is
      invariant under common positive scaling where representable, and zero
      still maps to identity; do not add a second normal-compatibility
      predicate. Independently compute extreme quaternion magnitude with a
      test-only `BigInteger` square-sum/root oracle: require the exact rounded
      finite magnitude when representable and `Fixed64.MaxValue` only when the
      true magnitude exceeds the public range.
- [ ] **Step 3: Add quaternion-log regressions** for normalized endpoint drift,
      exact `W = +/-One`, a zero quaternion, a tiny vector part, and extreme
      finite component combinations. Compare each nonzero result with the log of
      its independently normalized quaternion and assert no domain exception.
      Keep separate `FixedMath.Acos` tests proving values outside `[-1, 1]`
      remain invalid when no quaternion normalization contract applies.
- [ ] **Step 4: Add `BigInteger` degree/radian conversion oracles in tests.**
      Cover exact and adjacent round-half-to-even cases, ordinary values,
      intermediate-overflow/final-fit cases, and `Fixed64.MinValue`/
      `Fixed64.MaxValue`. Require `DegToRad` to return the representable oracle
      result and `RadToDeg` to saturate only when the final oracle result lies
      outside the asymmetric raw range.
- [ ] **Step 5: Replace old angle-rejection tests** with periodic-equivalence
      tests. Use radians offset by `+/-TwoPi` for radian constructors and degrees
      offset by `+/-360` for degree constructors, including multiple turns and
      both signs. Compare rotations modulo quaternion sign. Add `AngleAxis`
      parity with `FromAxisAngle(axis, FixedMath.DegToRad(angle))` and require a
      zero axis to return identity.
- [ ] **Step 6: Add extreme-axis and direction regressions** proving
      `FromAxisAngle` and `FromDirection` use scale-safe vector normalization
      rather than a saturating `MagnitudeSquared` sum.
- [ ] **Step 7: Run the focused tests and confirm** the new parity API is
      missing and the quaternion, conversion, and multi-turn regressions expose
      the current full-domain defects.

```powershell
dotnet test ../FixedMathSharp/tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter "FullyQualifiedName~Vector2d|FullyQualifiedName~FixedQuaternion|FullyQualifiedName~FixedTrigonometry"
```

- [ ] **Step 8: Implement `Vector2d.IsNormalized`** with the same nonzero and
      squared-magnitude epsilon contract as `Vector3d` and `Vector4d`.
- [ ] **Step 9: Replace quaternion magnitude/normalization arithmetic** with
      the proven scale-relative four-component pattern already used by
      `Vector4d`. Share a focused internal helper only if it removes real
      duplication without exposing a public wide-number abstraction.
- [ ] **Step 10: Route axis/direction normalization** through the scale-safe
      `Vector3d.Normalized` contract in `FromAxisAngle` and `FromDirection`.
      Preserve identity for a zero axis/direction and the current deterministic
      opposite-direction fallback.
- [ ] **Step 11: Repair `QuaternionLog` at its invariant boundary.** Normalize
      scale-safely, retain the existing tiny-vector early return, clamp `W`
      unconditionally to `[-1, 1]`, and then call strict `Acos`. Replace
      `0x00001000L` with a named decimal raw threshold `4_096` and document that
      it is approximately `9.536743e-7` in Q32.32.
- [ ] **Step 12: Remove the `[-Pi, Pi]` guards** from radian quaternion
      constructors. Route `DegToRad` and `RadToDeg` through Task 4's internal
      fused result core with one final rounding/saturation decision. Reduce
      `AngleAxis` to delegation through
      `FromAxisAngle(axis, FixedMath.DegToRad(angle))`; do not retain its
      duplicate axis-angle implementation or add a public angle wrapper.
- [ ] **Step 13: Run the focused tests in `Release` and `ReleaseLean`, then run
      exact FixedMathSharp coverage.** Cover every zero, scale selection,
      endpoint clamp, final conversion saturation, periodic reduction, and
      `IsNormalized` branch.
- [ ] **Step 14: Benchmark the new `Vector2d.IsNormalized`, `DegToRad`, and
      `RadToDeg` rows plus existing quaternion magnitude/normalize/construct
      rows.** Require zero allocations and no material ordinary-input
      regression; optimize shared fused/scale-relative paths rather than
      restoring saturating duplicate arithmetic.
- [ ] **Step 15: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

---

### Task 6: Full-Domain Fixed Segment Geometry Ownership

**Files:**

- Modify: `../FixedMathSharp/src/FixedMathSharp/Core/FixedMath.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Scalars/Fixed64.Operators.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector2d.Statics.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedSegment2d.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedSegment.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector3d.Statics.cs`
- Modify: `../FixedMathSharp/docs/wiki/bounds-and-geometry.md`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Core/FixedMath.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Scalars/Fixed64.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Geometry/Primitives/FixedSegment2d.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Geometry/Primitives/FixedSegment.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs`

**Interfaces:**

- Consumes: Tasks 1 through 4 for consistent rounding and the internal unsigned
  word arithmetic used by full-width multiplication and accumulation.
- Produces:

```csharp
public partial struct FixedSegment2d
{
    public readonly bool TryGetUniqueIntersection(
        FixedSegment2d other,
        out Fixed64 thisParameter);

    public readonly (Vector2d ThisPoint, Vector2d OtherPoint) GetClosestPoints(
        FixedSegment2d other);
}

public partial struct FixedSegment
{
    public readonly (Vector3d ThisPoint, Vector3d OtherPoint) GetClosestPoints(
        FixedSegment other);
}
```

- Unique intersection returns `false` with `thisParameter = default` for
  disjoint or positive-length collinear-overlap cases. A single shared endpoint
  is a unique intersection. A zero-length segment is a point: a point on the
  other segment, including identical point segments, is a unique intersection
  with this segment parameter zero; distinct points and off-segment points are
  not.
- Closest-pair results always lie on their respective finite segments.
  Degenerate segments are points; equal-distance ties keep the first candidate
  in the documented deterministic order.
- Existing `FixedMath.Lerp` and `Vector2d.ClosestPointOnLineSegment` keep their
  signatures but gain full-domain endpoint-difference/interpolation behavior.
  Public distance-returning methods preserve `Fixed64` final saturation; the
  internal candidate comparison does not use that saturated value for ordering.
- `FixedSegment.GetClosestPoints` preserves the current ordinary finite-segment
  algorithm and result contract while handling a first, second, or both
  segments whose ordinary Q32.32 squared length resolves to zero symmetrically.
  This includes exact point segments and sub-square-root-resolution deltas. It
  does not acquire the 2D path's new full-domain guarantee in this workstream.
- No public arbitrary-precision, rational, determinant, or intersection-result
  hierarchy is introduced.

- [ ] **Step 1: Add 2D unique-intersection red tests** for interior crossing,
      both endpoint orders, endpoint touch, disjoint lines, parallel segments,
      collinear disjoint/touching/positive overlap, identical points, distinct
      points, one point lying on the other segment, and one off-segment point.
      Assert the first-segment parameter and default failure output.
- [ ] **Step 2: Add 2D closest-pair red tests** for crossing, parallel,
      collinear overlap, both degenerate, either degenerate, endpoint clamps,
      reversed endpoints, and deterministic equal-distance ties.
- [ ] **Step 3: Move the existing 3D closest-pair behavior tests** from
      `Vector3d.Tests` to `FixedSegment.Tests`, then add identical/distinct point
      segments, exactly degenerate first and second segments, one-raw-unit first
      and second segments whose squared length resolves to zero, reversed
      inputs, and symmetry regressions. Confirm the current vector helper throws
      for the degenerate-second case before changing source.
- [ ] **Step 4: Add full-domain point-projection and interpolation tests.** Use
      endpoints near opposite `Fixed64` limits so their raw delta requires 65
      bits, and cover midpoint ties plus parameters at zero and one. Require
      `FixedMath.Lerp`, `Vector2d.ClosestPointOnLineSegment`, and
      `FixedSegment2d.ClosestPoint` to return in-bounds oracle values without an
      early saturated delta.
- [ ] **Step 5: Add test-only `BigInteger` oracles** for orientation,
      determinant/numerator ratios, interpolated points, and exact squared
      distance ordering. Cover cancelling products, a smallest-representable
      nonzero determinant, endpoint coordinates near both limits, and cases
      where two public `DistanceSquared` results both saturate but the closer
      candidate is still uniquely knowable.
- [ ] **Step 6: Run the focused tests and confirm** the new segment APIs are
      missing and the extreme projection/ordering cases expose saturation in
      the existing implementation.
- [ ] **Step 7: Extend the internal wide arithmetic core** with the minimum
      allocation-free operations required here. Reuse Task 3's signed 65-bit
      raw endpoint differences, add 65-by-65-bit products and signed 192-bit
      cross/dot accumulation, exact magnitude comparison, and conversion of a
      proven unit-interval wide numerator/denominator ratio to Q32.32 using 32
      fractional bits plus guard/sticky round-half-to-even state. Keep these
      implementation details internal and shared with Tasks 3 and 4.
- [ ] **Step 8: Harden `FixedMath.Lerp`** using the 65-bit endpoint difference
      and a full-width difference-by-Q32.32 interpolation followed by one final
      round-half-to-even conversion. Do not implement it as
      `start + ((end - start) * amount)` with saturating intermediates.
- [ ] **Step 9: Harden `Vector2d.ClosestPointOnLineSegment`** by computing the
      projection numerator and squared-length denominator in the wide core,
      classifying/clamping the exact ratio before conversion, and interpolating
      through the hardened `FixedMath.Lerp` path. Zero-length segments return
      the start point exactly.
- [ ] **Step 10: Implement unique intersection** with exact wide determinant and
      numerator sign/range comparisons. Classify only an exactly zero
      determinant as parallel/collinear; do not use a physics epsilon. Handle
      the explicit point and collinear endpoint semantics before rejecting
      positive-length overlap, then convert only the proven `[0, 1]` parameter.
- [ ] **Step 11: Implement 2D closest points** by first accepting a unique
      intersection, then evaluating the four endpoint projections in a fixed
      order. Compare their squared raw distances in the wide accumulator rather
      than via saturating public `Fixed64` distances.
- [ ] **Step 12: Move the finite 3D solver into `FixedSegment.GetClosestPoints`.**
      Compute the existing `a`/`c` squared lengths first and classify either
      `== Fixed64.Zero` as a point at Q32.32 resolution before determinant or
      division arithmetic. Handle the first, second, and both-point cases
      symmetrically, keep the existing ordinary/parallel parameter policy,
      update every FixedMathSharp caller, and delete
      `Vector3d.ClosestPointsOnTwoLines` plus its private helpers. Do not add a
      forwarding compatibility method.
- [ ] **Step 13: Run segment, scalar, math, and vector tests in `Release` and
      `ReleaseLean`, then exact coverage.** Every point-degenerate, parallel,
      collinear, endpoint, determinant-sign, ratio-rounding, and tie-order branch
      must be covered.
- [ ] **Step 14: Benchmark 2D point projection, unique intersection, and closest
      pairs** in `BoundsBenchmarks`. Move the existing 3D closest-pair benchmark
      out of `Vector3dBenchmarks` and compare the same fixture through
      `FixedSegment`. Require zero allocations and no material ordinary-input
      regression; optimize shared word operations rather than adding a
      reduced-range fast path with different semantics.
- [ ] **Step 15: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

---

### Task 7: Explicit X/Z Planar Transform Contract

**Files:**

- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Matrices/FixedTransform.cs`
- Modify: `../FixedMathSharp/docs/wiki/coordinate-conventions.md`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Matrices/FixedTransform.Tests.cs`

**Interfaces:**

- Consumes: Task 5's multi-turn quaternion creation and the existing
  `Vector2d.ToVector3d`/`Vector3d.ToVector2d` X/Z embedding.
- Produces:

```csharp
public class FixedTransform
{
    public FixedTransform(
        Vector2d positionXZ,
        Fixed64 rotationXZRadians,
        Vector2d scaleXZ,
        FixedTransform? parent = null);

    public Vector2d PositionXZ { get; set; }
    public Fixed64 RotationXZRadians { get; set; }
    public Vector2d ScaleXZ { get; set; }
}
```

- The planar constructor uses elevation zero and Y scale one. `PositionXZ` and
  `ScaleXZ` setters preserve existing Y values. `RotationXZRadians` setter
  replaces the complete rotation with a pure rotation around Y using the
  negated planar angle.
- `FixedTransform` stores explicit `_position`, `_rotation`, and `_scale`
  components. The component constructor preserves negative and zero scale
  exactly. The matrix constructor decomposes once using the documented
  `Fixed4x4` convention; later component access does not repeatedly decompose
  or rebuild a hidden matrix.
- The misleading `LossyScale` alias is removed. Without parent composition it
  is neither a hierarchy-derived nor lossy world scale; downstream callers use
  `Scale`.
- `RotationXZRadians` getter reports the signed angle from embedded
  `Vector2d.Right` toward the transform's projected local-right direction. Pure
  X/Z transforms round-trip modulo `TwoPi`; the projection contract remains
  deterministic for a transform that also contains pitch or roll.

- [ ] **Step 1: Add constructor and component red tests** covering default Y
      elevation/scale, parent preservation, position and scale mutation while Y
      remains unchanged, negative and zero scale preservation, and rotation
      setter replacement of pitch/roll.
- [ ] **Step 2: Add basis-parity red tests.** Assert zero maps local right to
      embedded `Vector2d.Right`, `HalfPi` maps it to embedded
      `Vector2d.Forward`, and `-HalfPi` maps it oppositely. Compare against
      `Vector2d.Rotate` rather than a game-engine convention.
- [ ] **Step 3: Add rotation round-trip tests** for ordinary values, `+/-Pi`,
      `+/-TwoPi`, several positive/negative turns, negative/zero scale, and a
      quaternion containing pitch/roll. Document and assert the
      projected-local-right result for the nonplanar case, independent of scale.
- [ ] **Step 4: Add matrix-constructor decomposition tests** for translation,
      rotation, and supported signed-scale conventions. Document ambiguous or
      non-decomposable matrix behavior explicitly; component-constructed
      transforms must never depend on that ambiguity.
- [ ] **Step 5: Run `FixedTransformTests` and confirm the planar surface is
      missing.**
- [ ] **Step 6: Replace matrix-backed component storage** with explicit
      position, normalized rotation, and scale fields. Make existing component
      properties direct deterministic accessors, keep `Parent` reference
      semantics, perform matrix decomposition only in the matrix constructor,
      and remove `LossyScale`. Do not add a public matrix cache or retain two
      mutable sources of truth.
- [ ] **Step 7: Implement the planar constructor and position/scale properties**
      by reusing the existing X/Z conversion helpers. Do not add `Translate2D`,
      a plane enum, a second transform type, or a factory wrapping this
      constructor.
- [ ] **Step 8: Implement rotation parity** with a negated Y-axis quaternion in
      the setter and
      `Atan2(Rotation.Rotate(Vector3d.Right).Z, Rotation.Rotate(Vector3d.Right).X)`
      in the getter. Define a zero planar projection as zero radians; do not
      derive rotation from scaled matrix basis vectors or read degrees through
      `EulerAngles` on this path.
- [ ] **Step 9: Run `FixedTransformTests` and the full FixedMathSharp suite in
      `Release` and `ReleaseLean`, then exact coverage.** No benchmark is
      required for component projection and assignment unless downstream
      measurement shows a regression.
- [ ] **Step 10: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

---

### Task 8: Gravitas Consumes FixedMathSharp Arithmetic

**Files:**

- Delete: `src/Gravitas/Support/FixedVectorDifference.cs`
- Delete: `src/Gravitas/CollisionHandling/Detection/3D/ConvexSupportProjection.cs`
- Modify: `src/Gravitas/CollisionHandling/Continuous/ContinuousCollisionImpulsePolicy.cs`
- Modify: `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Dynamic.cs`
- Modify: `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Kinematic.cs`
- Modify: `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Dynamic.cs`
- Modify: `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Kinematic.cs`
- Modify: every current caller returned by:

```powershell
rg -l "FixedVectorDifference|ConvexSupportProjection" src/Gravitas tests/Gravitas.Tests
```

- Modify: `src/Gravitas/Queries/GjkSimplexScale.cs`
- Test: `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionMathTests.cs`
- Test: `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionPolicyTests.cs`
- Test: `tests/Gravitas.Tests/CollisionHandling/ConvexColliderSupportTests.cs`
- Test: `tests/Gravitas.Tests/Queries/GjkSimplexScaleTests.cs`
- Test: relevant 2D, 3D, and mixed query/CCD suites already exercising those
  callers.

**Interfaces:**

- Consumes: Tasks 1 through 4.
- Produces: no new Gravitas arithmetic API.
- Replaces the internal `ResolveVelocityDelta` overloads with component-atomic
  `TryResolveVelocityDelta` overloads. Failure writes `default` and occurs
  before either collision participant is mutated.

```csharp
internal static bool TryResolveVelocityDelta(
    Vector2d normal,
    Fixed64 responseSpeed,
    Fixed64 bodyInverseMass,
    Fixed64 constrainedInverseMass,
    out Vector2d velocityDelta);

internal static bool TryResolveVelocityDelta(
    Vector3d normal,
    Fixed64 responseSpeed,
    Fixed64 bodyInverseMass,
    Fixed64 constrainedInverseMass,
    out Vector3d velocityDelta);
```

- [ ] **Step 1: Add the odd-raw negative-expansion regression** using
      `MaxValue`, `MinValue`, and `Fixed64.MinIncrement`. Assert that the
      selected working shift makes the actual three-term support difference
      exact.
- [ ] **Step 2: Run `GjkSimplexScaleTests` and confirm the regression fails.**
- [ ] **Step 3: Replace endpoint and translation checks** with
      `Vector2d.TryAdd`/`TrySubtract` and `Vector3d.TryAdd`/`TrySubtract`, then
      delete `FixedVectorDifference` and its direct helper tests.
- [ ] **Step 4: Replace support ordering and conservative projection** with the
      FixedMathSharp vector APIs, then delete `ConvexSupportProjection` and its
      duplicated multiplier.
- [ ] **Step 5: Remove raw overflow predicates from `GjkSimplexScale`.** Keep
      the private bounds predicates, but implement them by composing FixedMathSharp
      `Try*` operations.
- [ ] **Step 6: Preserve GJK-owned power-of-two coordinate policy.** Use the
      existing `Fixed64 >>` operator for components. For the nonnegative radius
      bound, reconstruct the shifted value and add `Fixed64.MinIncrement` when
      discarded raw bits require a ceiling; this must conservatively cover
      negative arithmetic shifts.
- [ ] **Step 7: Reject shifts outside `0..2` explicitly** before any shift or
      restore operation. Do not add a new public enum or abstraction for an
      internal three-value implementation detail.
- [ ] **Step 8: Run focused GJK, support, query, and CCD tests in `Release` and
      `ReleaseLean`.**
- [ ] **Step 9: Delete the arithmetic-driven `IsResolvableMobility` cutoff** and
      replace both `ResolveVelocityDelta` overloads with component-atomic
      `TryResolveVelocityDelta` methods composed from the three-factor
      `TryMultiplyDivide` overload. For two-body response, compute both velocity
      deltas successfully before applying either one. Do not retain a
      solver-conditioning threshold without separate physical evidence.
- [ ] **Step 10: Add 2D and 3D CCD regressions** for the `65536` finite response,
      zero inverse mass, final-result overflow rejection without partial body
      mutation, and unchanged ordinary mobility. Run the focused
      continuous-collision policy and sweep suites.
- [ ] **Step 11: Owner review checkpoint.** Leave Gravitas implementation changes
      unstaged and provide a proposed commit message.

---

### Task 9: Gravitas Consumes FixedMathSharp Geometry And Planar Contracts

**Files:**

- Delete: `src/Gravitas/CollisionHandling/Detection/Geometry/PlanarSegmentGeometry.cs`
- Delete: `tests/Gravitas.Tests/CollisionHandling/PlanarSegmentGeometryTests.cs`
- Modify: every current caller returned by:

```powershell
rg -l "PlanarSegmentGeometry|ClosestPointsOnSegments|ClosestPointsOnTwoLines|LossyScale" src/Gravitas tests/Gravitas.Tests tests/Gravitas.Benchmarks
```

- Modify: `src/Gravitas/Core/2D/SolidBody2D.cs`
- Modify: `src/Gravitas/Core/2D/SolidBody2D.Motion.cs`
- Modify: `src/Gravitas/Core/2D/SolidBody2D.Serialization.cs`
- Modify: `src/Gravitas/Colliders/2D/LSCollider2D.cs`
- Modify: `src/Gravitas/Colliders/2D/LSCompoundCollider2D.cs`
- Modify: `src/Gravitas/Colliders/3D/LSCollider.cs`
- Modify: `src/Gravitas/Colliders/3D/LSMeshCollider.cs`
- Modify: `src/Gravitas/Colliders/3D/LSCompoundCollider.cs`
- Modify: `src/Gravitas/Constraints/3D/JointSolver3D.cs`
- Modify: `docs/wiki/DIMENSIONS.md`
- Modify: `docs/wiki/HOST_INTEGRATION.md`
- Test: `tests/Gravitas.Tests/Physics2D/SolidBody2DHostContractTests.cs`
- Test: `tests/Gravitas.Tests/Physics2D/Collider2DStateParityTests.cs`
- Test: `tests/Gravitas.Tests/Physics2D/Physics2DSimulationTests.cs`
- Test: relevant 2D/3D primitive, compound, and physics-mesh scale suites.
- Test: relevant 2D/mixed segment collision/query and 3D constraint stress suites.

**Interfaces:**

- Consumes: Tasks 5 through 7.
- Produces: no new Gravitas math API. Host transform synchronization uses
  `PositionXZ` and `RotationXZRadians`; segment operations use
  `FixedSegment2d` and `FixedSegment`; joint angular error uses
  `FixedQuaternion.QuaternionLog` directly; scale consumers use `Scale`.
- FixedTransform itself permits signed/zero scale, but Gravitas admits only
  strictly positive collider scale on every consumed axis. Invalid standalone
  or compound scale fails explicitly before bounds, radius, inertia, mesh, or
  partition state is built; no component-wise absolute-value fallback is used.
- Gravitas keeps its scalar 2D rotation canonical in the half-open interval
  `[-Pi, Pi)`, so `+Pi` has the single representative `-Pi`. This is
  authoritative state hygiene, not a second quaternion angle restriction.

- [ ] **Step 1: Add host/collision parity regressions** with an asymmetric 2D
      polygon or capsule. For `+HalfPi` and `-HalfPi`, assert the host transform's
      embedded local-right direction matches `Vector2d.Rotate`, and kinematic
      readback reconstructs the same scalar rotation.
- [ ] **Step 2: Add multi-turn 2D regressions** for initialization,
      `ResetPosition`, `SetRotation`, dynamic integration across `Pi`, kinematic
      host readback, serialization population, and repeated positive/negative
      turns. Assert no quaternion construction throws and authoritative state
      remains in `[-Pi, Pi)` while representing the requested rotation. Add
      exact representative assertions for `+Pi`, `-Pi`, and equivalent
      multi-turn values to protect deterministic state hashes.
- [ ] **Step 3: Add a joint-solver regression** using the exact quaternion-log
      endpoint-drift input from Task 5 through the public solver path. Confirm
      the local FixedMathSharp reference fixes it before deleting the Gravitas
      workaround.
- [ ] **Step 4: Add collider-scale admission regressions** for zero and negative
      components on each participating axis of standalone 3D primitives,
      meshes, 2D/3D compound parts, and runtime scale rebuilds. Require one
      explicit failure contract before shape/partition mutation; retain positive
      nonuniform-scale behavior. Prove FixedTransform still stores the rejected
      signed value so the policy boundary is Gravitas, not hidden math loss.
- [ ] **Step 5: Replace manual X/Z transform projection** in 2D body/collider
      host paths with `PositionXZ` and `RotationXZRadians`. Preserve host Y
      elevation and do not alter mixed-slab ownership. Replace every
      `LossyScale` consumer with explicit validated `Scale`; do not recreate the
      removed alias or take an implicit absolute value in Gravitas.
- [ ] **Step 6: Centralize collider-scale validation** at the earliest shared
      2D/3D standalone and compound admission/rebuild boundaries. Reject any
      consumed component `<= Fixed64.Zero` before mutating runtime shape,
      bounds, mass, mesh, or partition state. Reuse that contract from mesh
      validation instead of preserving shape-specific disagreement.
- [ ] **Step 7: Add one private scalar rotation canonicalizer** in
      `SolidBody2D` using remainder by `TwoPi`, then subtracting `TwoPi` when the
      result is `>= Pi` or adding `TwoPi` when it is `< -Pi`. Route every
      authoritative external assignment and simulation commit through it; do
      not canonicalize temporary CCD sample/restore values independently.
- [ ] **Step 8: Replace `GetSafeQuaternionLog`** with direct
      `FixedQuaternion.QuaternionLog`, delete the duplicate log implementation,
      and retain only solver-specific twist thresholds. Replace any remaining
      hexadecimal raw threshold with the named decimal value and units.
- [ ] **Step 9: Replace point projection and distance callers** with
      `FixedSegment2d.ClosestPoint`/`DistanceSquared`, then replace both private
      2D closest-pair solvers and unique-intersection callers with Task 6's
      segment methods. Delete `PlanarSegmentGeometry` and its duplicate tests;
      move behavior coverage to FixedMathSharp rather than retaining forwarding
      tests in Gravitas.
- [ ] **Step 10: Replace private 3D closest-segment wrappers** with
      `FixedSegment.GetClosestPoints` where their semantics match. Keep a local
      physics threshold only if a regression proves that threshold-length
      collider axes must intentionally be treated as points; document that
      physical policy beside the caller rather than adding another general
      geometry wrapper.
- [ ] **Step 11: Run focused 2D, 3D, mixed, segment, scale, constraint, and
      serialization tests in `Release` and `ReleaseLean`.** Confirm no
      `PlanarSegmentGeometry`, `GetSafeQuaternionLog`, or manual
      `EulerAngles.Y` 2D host mapping remains, and `rg -n "LossyScale"`
      returns no runtime or test caller.
- [ ] **Step 12: Run the existing 2D simulation, mixed collision/query, and 3D
      constraint benchmark rows.** Require zero allocation regression and no
      material slowdown from value-type segment construction or planar
      projection.
- [ ] **Step 13: Update coordinate documentation** with the explicit X/Z
      property names, positive-angle basis, Y preservation, canonical Gravitas
      scalar rotation range, and the distinction between general signed
      transform scale and strictly positive physics-collider dimensions.
- [ ] **Step 14: Owner review checkpoint.** Leave all Gravitas changes unstaged
      and provide a proposed commit message.

---

### Task 10: Remove Release-Only Assertion Behavior

**Files:**

- Modify: `src/Gravitas/Queries/3D/Sweeps/ConvexShape.cs`
- Modify: `src/Gravitas/CollisionHandling/Detection/3D/ConvexColliderSupport.cs`
- Test: `tests/Gravitas.Tests/Queries/GravitasQuery3DServiceSweepTests.cs`
- Test: `tests/Gravitas.Tests/CollisionHandling/ConvexColliderSupportTests.cs`

**Interfaces:**

- Produces: identical debug and release behavior for invalid internal shape
  use; no new public API.

- [ ] **Step 1: Add tests** proving triangle shapes cannot be used as sweep
      sources and circle slabs cannot be used as target-normal providers.
- [ ] **Step 2: Replace both tagged-union `Debug.Assert` guards** with
      `SwiftThrowHelper.ThrowIfTrue` and remove `System.Diagnostics`.
- [ ] **Step 3: Make zero-direction cone support deterministic** by selecting
      `Vector3d.Right`, matching the existing general support-mapping fallback,
      then remove the cone `Debug.Assert`.
- [ ] **Step 4: Run the focused tests in `Release` and `ReleaseLean`.**
- [ ] **Step 5: Confirm `rg -n "Debug\\.Assert" src/Gravitas` returns no
      matches.**

---

### Task 11: Re-Achieve 100% FixedMathSharp Coverage

**Files:**

- Modify: focused files under `../FixedMathSharp/tests/FixedMathSharp.Tests`
  selected from the fresh Cobertura gaps.
- Modify: focused files under
  `../FixedMathSharp/tests/FixedMathSharp.Chronicler.Tests` if the merged report
  exposes reachable adapter gaps.
- Modify: `../FixedMathSharp/src/FixedMathSharp` only when review proves a gap
  is unreachable stale code rather than missing behavior coverage.
- Modify: `../FixedMathSharp/docs/complexity-exceptions.md`
- Generate: `../FixedMathSharp/tests/FixedMathSharp.Tests/TestResults/coverage-analysis`

**Interfaces:**

- Consumes: all FixedMathSharp changes from Tasks 1 through 7.
- Produces: one fresh merged Cobertura/ReportGenerator artifact proving 100%
  reachable line, branch, and method coverage across the FixedMathSharp runtime
  package set. Generated and compiler-generated sources remain excluded by the
  existing runsettings; no behavioral source is excluded to manufacture the
  result.
- Start this task from the Gravitas root. Step 1 enters the sibling
  `FixedMathSharp` repository and the remaining commands stay there through the
  owner checkpoint, so solution, runsettings, report, and complexity paths
  resolve consistently.

- [ ] **Step 1: Capture the post-hardening baseline** from a clean results
      directory with the repository's existing runsettings:

```powershell
Push-Location ..\FixedMathSharp
dotnet test FixedMathSharp.slnx --configuration Debug --collect:"XPlat Code Coverage" --results-directory tests/FixedMathSharp.Tests/TestResults/coverage-analysis/raw --settings tests/FixedMathSharp.Tests/coverlet.runsettings --verbosity normal
```

- [ ] **Step 2: Merge every emitted Cobertura file** with ReportGenerator into
      `tests/FixedMathSharp.Tests/TestResults/coverage-analysis/reports`, emit
      HTML and text summaries, and record exact covered/total line, branch, and
      method counts before adding tests.
- [ ] **Step 3: Build a gap ledger ordered by owning source block.** Keep related
      branches together instead of jumping among files. Classify each gap as a
      reachable public behavior, an internal state reachable through a public
      operation, or mathematically/structurally unreachable stale code.
- [ ] **Step 4: Close reachable gaps with focused behavior tests.** Assert exact
      results, state transitions, exceptions, deterministic ordering, or
      serialization round trips. Do not use reflection when a public path
      exists, test-only production switches, empty assertions, or assertions
      that merely repeat setup values.
- [ ] **Step 5: Remove only proven unreachable branches** when their invariant
      is already enforced by the shared caller or type construction. Document
      the proof in the owning test or source comment only when it is not obvious;
      do not add pragma exclusions or defensive zombie branches for the metric.
- [ ] **Step 6: Re-run focused tests after each source block, then rerun the full
      Debug coverage command and merged report.** Continue until the report
      shows exactly 100% line, 100% branch, and 100% method coverage with zero
      test failures.
- [ ] **Step 7: Run coverage/CRAP analysis** against the final Cobertura file.
      Require no CRAP hotspot above 30 and update
      `docs/complexity-exceptions.md` only from the fresh method metrics.
- [ ] **Step 8: Run the complete `Release` and `ReleaseLean` suites** after
      coverage closure to prove coverage-only fixtures did not change package
      behavior or depend on generated MemoryPack code.

```powershell
dotnet test FixedMathSharp.slnx --configuration Release --no-restore
dotnet test FixedMathSharp.slnx --configuration ReleaseLean --no-restore
```

- [ ] **Step 9: Owner review checkpoint.** Leave all coverage tests, justified
      dead-code removals, documentation, and generated artifacts unstaged and
      report the final test/line/branch/method/CRAP counts with a proposed commit
      message. Return to the Gravitas repository with `Pop-Location` after
      capturing the evidence.

---

### Task 12: Cross-Stack Validation And Documentation Closure

**Files:**

- Modify: `docs/feature-work/issue-tracker.md`
- Modify: `docs/feature-work/feature-work-overview.md`
- Modify: `../FixedMathSharp/docs/complexity-exceptions.md` only if a new method
  exceeds the registered complexity threshold after coverage/CRAP analysis.
- Modify: relevant FixedMathSharp XML documentation for every new public API.
- Temporarily modify, then restore: dependency, test, and benchmark project
  references in `../SwiftCollections`, `../GridForge`, and Gravitas.

- [ ] **Step 1: Confirm Task 11's final merged artifact** still reports exact
      100% line, branch, and method coverage, then run clean FixedMathSharp
      `Release` and `ReleaseLean` package builds/tests from the reviewed tree.
- [ ] **Step 2: Run the focused FixedMathSharp benchmark rows** for scalar
      division, fused multiply-divide, vector `Try*`, magnitude/normalization,
      degree/radian conversion, quaternion construction/log, projection,
      segment geometry, and `Vector2d.IsNormalized`. Record medians and
      allocations.
- [ ] **Step 3: Validate SwiftCollections, GridForge, and Gravitas** through
      explicit local project references in each library, test, and benchmark
      project that requires them. Treat this as source-integration evidence, not
      package-release evidence.
- [ ] **Step 4: Complete the owner-controlled FixedMathSharp release
      checkpoint.** After local-source validation is accepted, restore any
      consumer package links needed for a clean FixedMathSharp package build,
      rerun its package-only `Release`/`ReleaseLean`/coverage gates, commit the
      reviewed changes, and release the new FixedMathSharp package before
      advancing a downstream manifest.
- [ ] **Step 5: Advance the lower consumers sequentially against released
      packages.** Replace SwiftCollections local links with the released
      FixedMathSharp version, restore/build/test/benchmark its package-only
      solution, then complete its owner release checkpoint. Repeat for
      GridForge against the released FixedMathSharp and SwiftCollections
      versions, but do not release GridForge until ordered issue 1,
      `GridForge Reuses Grid Spawn Tokens Across Pooled Generations`, is fixed
      and independently verified in its own reviewed change. Do not validate a
      later package against an unreleased local dependency and call that release
      closure.
- [ ] **Step 6: Restore Gravitas package references and run package-only
      gates.** Remove every Gravitas local project link, update package
      references to the released FixedMathSharp, SwiftCollections, and
      GridForge versions, and restore from packages. Then run full `Release`,
      `ReleaseLean`, exact coverage, replay, and the existing convex-sweep, 2D
      simulation, mixed collision, and constraint benchmark rows. Require 100%
      line and branch coverage, deterministic replay, and zero allocation
      regression.
- [ ] **Step 7: Update the issue tracker resolution record** with the arithmetic
      ownership correction, division-rounding RCA, exact reciprocal identities,
      full-domain quaternion normalization/conversion, quaternion-log endpoint
      repair, multi-turn quaternion contract, component-backed transform and
      explicit X/Z parity, full-domain segment-geometry ownership, odd-raw GJK
      regression, final test counts, coverage artifact, and benchmark evidence.
      Remove the previous claim that the staged downstream arithmetic was
      already the final ownership boundary.
- [ ] **Step 8: Request independent final review** of correctness,
      determinism, API ownership, hot-path cost, and documentation consistency.
- [ ] **Step 9: Move this plan to `docs/feature-work/done/`** and update the
      overview only after all source-linked validation, sequential releases,
      package-only gates, and independent review pass.

## Exit Criteria

- Gravitas contains no generic raw fixed-point overflow detector, full-width
  multiplier, or exact projection accumulator.
- `FixedVectorDifference`, `ConvexSupportProjection`, and
  `PositiveFixed64RawHighLimit` no longer exist in Gravitas.
- `Fixed64.operator /` and `FixedMath.FastDiv` share one magnitude-division and
  rounding core, match the `BigInteger` oracle, and round exact midpoints to the
  even raw value for both signs.
- `x / Two == x * Half`, `x / 4 == x * Quarter`, and
  `x / 8 == x * Eighth` hold across the tested raw domain including
  `long.MinValue` and `long.MaxValue`.
- Existing operators remain saturating; `Try*` methods report exactness without
  throwing or exposing partial results.
- Both fused multiply-divide overloads retain their complete numerator until one
  final round-half-to-even conversion, report only final representability, and
  allocate no managed memory.
- Gravitas no longer rejects the finite `65536` CCD response because an
  intermediate inverse-mass ratio would saturate.
- Projection comparison is correct for the complete `Fixed64` component domain,
  not only normalized axes.
- `Vector2d`, `Vector3d`, and `Vector4d` expose the same tested
  `IsNormalized` contract; normal-cache compatibility remains Gravitas policy.
- Every nonzero finite quaternion normalizes through scale-relative arithmetic;
  `QuaternionLog` accepts normalized endpoint drift without weakening strict
  `Acos`, and Gravitas contains no duplicate safe-log method.
- `DegToRad` and `RadToDeg` use fused full-domain conversion with one final
  rounding/saturation decision. Axis-angle and Euler quaternion creation accept
  deterministic multi-turn angles, `AngleAxis` shares the same root path, and
  extreme axes/directions normalize without saturated magnitude sums.
- `FixedTransform` stores explicit position, normalized rotation, and scale,
  removes the misleading `LossyScale` alias, and exposes `PositionXZ`,
  `RotationXZRadians`, and `ScaleXZ`. Negative/zero authored scale survives,
  positive planar rotation matches `Vector2d.Rotate`, Y position/scale are
  preserved, and Gravitas host presentation matches 2D collision geometry.
- `FixedMath.Lerp`, `Vector2d.ClosestPointOnLineSegment`, and `FixedSegment2d`
  handle 65-bit endpoint differences and exact wide comparison before final
  Q32.32 conversion. `FixedSegment2d` owns unique intersection and closest-pair
  geometry, `FixedSegment` owns symmetric finite 3D closest pairs including
  tiny deltas whose ordinary squared length resolves to zero,
  `Vector3d.ClosestPointsOnTwoLines` is removed, and Gravitas contains no
  `PlanarSegmentGeometry` or private equivalent wrapper where those primitives
  apply.
- Gravitas authoritative planar rotation uses the single half-open
  `[-Pi, Pi)` representation, including exact `+Pi -> -Pi` canonicalization.
- Gravitas rejects every nonpositive scale component consumed by a standalone
  or compound collider before shape, bounds, mass, mesh, or partition mutation;
  FixedTransform still preserves signed/zero host scale without hidden absolute
  conversion.
- The odd-raw negative-expansion shift regression passes without saturation.
- No `Debug.Assert` remains in Gravitas runtime source.
- FixedMathSharp re-achieves 100% reachable line, branch, and method coverage
  without behavioral exclusions or hollow tests; Gravitas retains its 100%
  line and branch gates, and all measured hot paths remain allocation-free.
- All temporary local project references remain unstaged and are removed before
  package-only release validation.
