# Diagnostics And Formatting Surface Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:brainstorming before changing public API shape,
> superpowers:test-driven-development before production or test changes,
> performance-optimization-engineer before accepting performance-motivated
> runtime changes, and superpowers:verification-before-completion before
> claiming a phase is complete. Steps use checkbox (`- [ ]`) syntax for
> tracking.

**Status:** Active

**Goal:** Define a clear diagnostics and formatting surface for human-readable
FixedMathSharp values without confusing it with deterministic runtime math,
serialization layout, raw-value representation, or hot-path APIs.

**Architecture:** Keep deterministic numeric operations on the owning runtime
types and `FixedMath`. Treat diagnostics output as a low-frequency, explicitly
named surface. Prefer allocation-free `TryFormat`-style APIs where they are
worth the complexity, but do not make hot deterministic code depend on
human-readable formatting helpers.

**Tech Stack:** `netstandard2.1` and `net8.0` runtime targets, xUnit,
BenchmarkDotNet when allocation or formatting cost matters, `Fixed64`,
vectors, quaternions, matrices, bounds, ranges, invariant culture formatting,
and engine-agnostic FixedMathSharp core APIs.

---

## Context

The public API hardening track intentionally left the `ToFormatted*` helpers as
a later ownership review target. Current formatting and diagnostics helpers are
spread across scalar extensions and several value types, which can blur three
separate concerns:

- deterministic representation and raw Q32.32 conversion.
- serialization shape for MemoryPack and JSON.
- human-readable diagnostics, logging, editor display, and debugging.

This plan keeps that work separate from the completed public API hardening
track so release migration notes can be gathered later from the full v4.0.1 to
v5.0.0 commit range.

## Recommended Transition Shape

`ISpanFormattable` is the modern .NET shape we eventually want, but it is not
available to `netstandard2.1`. The transition should preserve Unity-friendly
target support while making the eventual modern-.NET implementation nearly
mechanical:

- Add `IFormattable` and `ToString(string? format, IFormatProvider? provider)`
  where the method shape is useful for scalar and composite diagnostics.
- Add public `TryFormat(Span<char> destination, out int charsWritten,
  ReadOnlySpan<char> format, IFormatProvider? provider)` methods now, even on
  `netstandard2.1`, without implementing `ISpanFormattable` on that target.
- Conditionally implement `ISpanFormattable` for `net8.0` using partials or
  guarded declarations once the public method shape is proven.
- Keep `double`-backed formatting clearly scoped to human-readable diagnostics.
  It is convenient and familiar, but it is not exact serialization and should
  not be used to recover deterministic fixed-point values.
- Preserve exact/raw output through explicit APIs such as `ToRawString` and
  `FromRaw`. If exact decimal text becomes a requirement, design a deterministic
  integer-based formatter instead of relying on floating-point conversion.

## Guardrails

- Do not treat diagnostic formatting as serialization. MemoryPack and JSON
  roundtrip behavior remains a separate compatibility concern.
- Do not let formatting helpers introduce allocations into deterministic hot
  paths that currently avoid them.
- Use invariant, explicit formatting rules unless a caller-provided culture or
  format provider is intentionally part of the API.
- Keep raw-value APIs explicit: `FromRaw`, `m_rawValue`, `ToRawString`, and
  integer-value conversions must remain semantically distinct.
- Do not require `ISpanFormattable` on `netstandard2.1`; expose same-shaped
  `TryFormat` methods there and reserve interface implementation for compatible
  target frameworks.
- Preserve `netstandard2.1` compatibility. If a formatting API needs a newer
  feature, isolate or avoid it.

## Phase 1: Formatting Inventory And Ownership Map

**Files:**

- Review: `src/FixedMathSharp/Numerics/Scalars/Fixed64.Extensions.cs`
- Review: `src/FixedMathSharp/Numerics/Scalars/Fixed64.cs`
- Review: `src/FixedMathSharp/Numerics/Vectors/*.cs`
- Review: `src/FixedMathSharp/Numerics/Rotations/*.cs`
- Review: `src/FixedMathSharp/Numerics/Matrices/*.cs`
- Review: `src/FixedMathSharp/Geometry/**/*.cs`
- Modify: this plan and, if needed, `docs/feature-work/issue-tracker.md`

- [ ] Inventory public `ToString`, `ToFormattedString`, `ToFormattedDouble`,
  `ToFormattedFloat`, `ToRawString`, and any formatting-like helpers.
- [ ] Classify each API as representation, diagnostics, serialization,
  convenience, or legacy/remove.
- [ ] Identify formatting call sites that allocate in valid hot paths.
- [ ] Identify culture-sensitive formatting risks and default formatting
  assumptions.
- [ ] Decide whether diagnostics formatting belongs in extensions, a dedicated
  static surface, or targeted instance methods.

Verification:

```bash
rg -n "ToFormatted|ToRawString|ToString\\(|IFormatProvider|TryFormat|Format" src/FixedMathSharp tests/FixedMathSharp.Tests
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Debug -f net8.0 --no-restore
```

## Phase 2: API Shape And Naming Decision

**Files:**

- Modify as needed: scalar/vector/quaternion/matrix/bounds formatting files
- Modify: matching tests under `tests/FixedMathSharp.Tests`
- Modify: README or wiki docs if public guidance changes

- [ ] Decide whether `ToFormatted*` should remain, be renamed, or move to a
  dedicated diagnostics surface.
- [ ] Decide whether raw formatting should remain on `Fixed64` only or be
  exposed consistently through diagnostics helpers.
- [ ] Adopt a same-shaped `TryFormat` method contract where formatting belongs,
  while keeping `ISpanFormattable` implementation conditional for compatible
  target frameworks.
- [ ] Decide whether `double`-backed diagnostic formatting is acceptable for
  each type, or whether a type requires an exact/raw formatting path.
- [ ] Preserve convenient debugging output while making hot-path costs obvious.
- [ ] Document why the chosen shape is better than keeping formatting spread
  across ad-hoc extensions.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Fixed64|FullyQualifiedName~Vector|FullyQualifiedName~Quaternion|FullyQualifiedName~Matrix|FullyQualifiedName~Bounds"
git diff --check
```

## Phase 3: Implementation And Compatibility Sweep

**Files:**

- Modify: accepted runtime API files
- Modify: tests and docs for any public rename or removal
- Review: `docs/wiki/fixed64-representation.md`

- [ ] Implement the chosen diagnostics/formatting surface.
- [ ] Add `IFormattable` and `ToString(string? format, IFormatProvider?
  provider)` where applicable.
- [ ] Add public `TryFormat(Span<char>, out int, ReadOnlySpan<char>,
  IFormatProvider?)` methods without depending on `ISpanFormattable` for
  `netstandard2.1`.
- [ ] Add conditional `ISpanFormattable` implementation for `net8.0` if the
  same-shaped methods prove clean and useful.
- [ ] Remove or obsolete rejected formatting helpers if the major-release
  cleanup allows it.
- [ ] Add tests for invariant formatting, rounding expectations, raw-value
  output, negative values, min/max-adjacent values, and vector/matrix composite
  formatting.
- [ ] Confirm serialization tests are unaffected by diagnostics formatting
  changes.
- [ ] Update docs that explain raw representation versus human-readable
  formatting.

Verification:

```bash
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean -f netstandard2.1 --no-restore
git diff --check
```

## Phase 4: Allocation And Reporting Review

**Files:**

- Modify if useful: `tests/FixedMathSharp.Benchmarks`
- Modify if useful: `tests/FixedMathSharp.Benchmarks/README.md`

- [ ] Add benchmarks only for formatting APIs that are likely to be used in
  repeated diagnostics, editor display, or logging loops.
- [ ] Capture allocation behavior for string-returning APIs versus any
  span-based alternatives.
- [ ] Keep formatting benchmark results separate from deterministic math
  throughput claims.
- [ ] Add final docs guidance that explains MemoryPack, JSON, raw values, and
  diagnostic formatting as separate concerns.

Verification:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj --configuration Release -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll list
git diff --check
```
