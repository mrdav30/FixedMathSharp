# Coordinate Convention Battle Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:test-driven-development before production or test changes, and use superpowers:verification-before-completion before claiming a phase is complete. Steps use checkbox (`- [ ]`) syntax for tracking.

**Status:** Proposed

**Goal:** Make FixedMathSharp's coordinate and forward-axis conventions explicit, tested, and adapter-friendly without binding the core library to Unity, XNA, MonoGame, or any other engine.

**Architecture:** Keep the core math package canonical and engine-agnostic: `+X` is right, `+Y` is up, and `+Z` is the library's default forward direction. Add named convention helpers only where they remove ambiguity, and push engine-specific terminology or axis flips into integration packages and docs.

**Tech Stack:** `netstandard2.1` and `net8.0` runtime targets, xUnit, `Fixed64`, `Vector2d`, `Vector3d`, `Vector4d`, `FixedQuaternion`, `Fixed3x3`, `Fixed4x4`, FixedMathSharp-Unity adapter extensions.

---

## Context

`Vector3d.Forward` currently returns `(0, 0, 1)` and `Vector3d.Backward` returns `(0, 0, -1)`. That matches Unity's semantic forward/back directions, but is reversed from XNA/MonoGame naming. The repo already has broad behavior built around `+Z` forward, including `Vector3d.Direction`, `FixedQuaternion.FromDirection`, `FixedQuaternion.LookRotation`, `Fixed4x4.CreateLookAt`, `Fixed4x4.CreateWorld`, matrix axis extraction, rays, planes, and frustum tests.

The important distinction is that FixedMathSharp should define a deterministic math convention, not an engine convention. Engine packages should convert between engine semantics and the core convention at boundaries.

## Risks To Avoid

- Do not flip `Vector3d.Forward` or `Vector3d.Backward` in core as a compatibility shortcut. That would silently change rotations, look matrices, rays, planes, bounds tests, and serialized intent.
- Do not add Unity/XNA conditionals to the core package.
- Do not add a global mutable convention setting. Deterministic simulation should not depend on process-wide runtime state.
- Do not solve this only with comments. The convention needs tests and conversion APIs where ambiguity can affect behavior.
- Do not introduce allocations or high-complexity adapter paths in hot vector, quaternion, or matrix operations.

## Phase 1: Convention Inventory And Guardrail Tests

**Files:**

- Modify: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector4d.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Numerics/Rotations/FixedQuaternion.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Numerics/Matrices/Fixed4x4.Tests.cs`
- Review: `docs/feature-work/issue-tracker.md`

- [ ] Add explicit tests that document the core 3D basis: `Vector3d.Right == (1, 0, 0)`, `Vector3d.Up == (0, 1, 0)`, `Vector3d.Forward == (0, 0, 1)`, and `Vector3d.Backward == (0, 0, -1)`.
- [ ] Add cross-product orientation tests that state `Vector3d.Cross(Vector3d.Right, Vector3d.Up) == Vector3d.Forward`.
- [ ] Add matrix basis tests that prove `Fixed4x4.Identity.Forward == Vector3d.Forward` and `Fixed4x4.CreateWorld(position, Vector3d.Forward, Vector3d.Up)` preserves the canonical axes.
- [ ] Add quaternion tests that prove `FixedQuaternion.LookRotation(Vector3d.Forward)` is identity and `FixedQuaternion.FromDirection(Vector3d.Forward)` is identity.
- [ ] Do not fix known direction-helper bugs in this phase. Track and resolve those through `docs/feature-work/issue-tracker.md`.
- [ ] Add 2D convention tests that distinguish `Vector2d.Forward == (0, 1)` from `Vector2d.ForwardDirection(Fixed64.Zero) == Vector2d.Right`.
- [ ] Add `Vector4d` tests only for component-preserving conversion behavior, not forward/back semantics.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Vector3dTests|FullyQualifiedName~Vector2dTests|FullyQualifiedName~Vector4dTests|FullyQualifiedName~FixedQuaternionTests|FullyQualifiedName~Fixed4x4Tests"
```

## Phase 2: Add Engine-Agnostic Convention Types

**Files:**

- Create: `src/FixedMathSharp/Numerics/CoordinateConventions/ForwardAxis.cs`
- Create: `src/FixedMathSharp/Numerics/CoordinateConventions/CoordinateConvention3d.cs`
- Create: `tests/FixedMathSharp.Tests/Numerics/CoordinateConventions/CoordinateConvention3d.Tests.cs`
- Modify: `src/FixedMathSharp/FixedMathSharp.csproj` only if the project uses explicit compile item lists.

- [ ] Add a small `ForwardAxis` enum with `PositiveZ` and `NegativeZ`.
- [ ] Add a readonly `CoordinateConvention3d` value type with predefined `PositiveZForward` and `NegativeZForward` instances.
- [ ] Expose `Forward`, `Backward`, `Right`, `Left`, `Up`, and `Down` from the convention type.
- [ ] Add conversion helpers that map direction vectors between a convention and the core canonical convention. The `PositiveZ` convention should be identity; the `NegativeZ` convention should flip `Z`.
- [ ] Keep helpers explicit and stateless. Do not add thread-static, static mutable, or ambient convention state.
- [ ] Add tests proving round trips between `PositiveZForward`, `NegativeZForward`, and canonical vectors.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~CoordinateConvention3dTests"
```

## Phase 3: Clarify Core Documentation And XML Comments

**Files:**

- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector3d.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector2d.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector4d.cs`
- Modify: `src/FixedMathSharp/Numerics/Rotations/FixedQuaternion.cs`
- Modify: `src/FixedMathSharp/Numerics/Matrices/Fixed4x4.cs`
- Modify: `README.md`
- Create or modify: `docs/wiki/coordinate-conventions.md`

- [ ] Replace `// TODO: in XNA this is reverse` comments with XML docs that state FixedMathSharp's canonical convention.
- [ ] Document that XNA/MonoGame use opposite forward naming and should convert at adapter boundaries.
- [ ] Document that Unity's forward naming aligns with core `+Z`, but Unity matrix storage and transform semantics still require semantic conversion helpers.
- [ ] Document `Vector2d` as plane math where `Forward == +Y`, and call out that `ForwardDirection(0)` returns `+X` because it is polar angle math.
- [ ] Document `Vector4d` as component and homogeneous-coordinate math, not a type with independent forward/back semantics.
- [ ] Add README links to the coordinate-conventions wiki page without making the README crowded.

Verification:

```bash
dotnet build FixedMathSharp.slnx --configuration Debug
```

## Phase 4: Update Unity Adapter Semantics

**Files:**

- Modify: `com.mrdav30.fixedmathsharp/Runtime/Extensions/Vector3d.Extensions.cs`
- Modify: `com.mrdav30.fixedmathsharp/Runtime/Extensions/Vector2d.Extensions.cs`
- Modify: `com.mrdav30.fixedmathsharp/Runtime/Extensions/FixedQuaternion.Extensions.cs`
- Modify: `com.mrdav30.fixedmathsharp/Runtime/Extensions/Fixed4x4.Extensions.cs`
- Mirror equivalent changes under `com.mrdav30.fixedmathsharp.lean/Runtime/Extensions/`
- Modify package README files as needed.

- [ ] Add XML comments that these Unity conversions use FixedMathSharp's canonical `+Z` forward convention.
- [ ] Keep Unity vector conversions as direct component mappings unless tests prove a semantic mismatch.
- [ ] Review quaternion conversions against basis tests. Direct component mapping is acceptable only if Unity-facing tests prove equivalent forward/up/right behavior.
- [ ] Review matrix conversion helpers separately from vector direction conventions. Matrix storage remapping is already semantic and should stay explicit.
- [ ] Add Unity package tests or sample assertions where available for `Vector3.forward`, `Quaternion.LookRotation(Vector3.forward)`, and `Matrix4x4.TRS` round-trip behavior.

Verification:

```bash
dotnet test FixedMathSharp.slnx --configuration Debug
```

Also verify the Unity package through its established Unity package workflow before publishing package changes.

## Phase 5: Add XNA/MonoGame Adapter Guidance Or Package

**Files:**

- Create or modify only after deciding whether XNA/MonoGame support belongs in docs, a separate package, or consumer sample code.
- Likely docs target: `docs/wiki/coordinate-conventions.md`
- Optional package target: a separate engine adapter project, not `src/FixedMathSharp`.

- [ ] Document XNA/MonoGame as a `NegativeZForward` semantic convention.
- [ ] Provide sample conversion code that flips `Z` for positions/directions when crossing between XNA/MonoGame semantic space and FixedMathSharp canonical space.
- [ ] Treat matrices and quaternions as semantic conversions, not blind component copies, unless tests prove equivalence for the selected convention.
- [ ] Keep any XNA/MonoGame references out of the core runtime types except in docs that explain convention differences.

Verification:

```bash
dotnet build FixedMathSharp.slnx --configuration Debug
```

## Phase 6: Full Regression Pass

**Files:**

- Review: `src/FixedMathSharp/**/*.cs`
- Review: `tests/FixedMathSharp.Tests/**/*.cs`
- Review: `README.md`
- Review: `docs/wiki/coordinate-conventions.md`
- Review: Unity package extension files if Phase 4 is included.

- [ ] Run the standard debug workflow.
- [ ] Run release and lean test configurations.
- [ ] Search for stale `TODO`, `XNA`, `Unity`, `Forward`, `Backward`, `Direction`, and `convention` text that contradicts the canonical convention.
- [ ] Review `docs/feature-work/issue-tracker.md` and confirm known direction-helper issues are either resolved or intentionally deferred.
- [ ] Confirm no public API breaking change was introduced unless a release note and migration note are prepared.
- [ ] Confirm serialization layouts are unchanged.

Verification:

```bash
dotnet restore
dotnet build FixedMathSharp.slnx --configuration Debug --no-restore
dotnet test FixedMathSharp.slnx --configuration Debug
dotnet test FixedMathSharp.slnx --configuration Release --no-restore
dotnet test FixedMathSharp.slnx --configuration ReleaseLean --no-restore
rg -n "TODO: in XNA this is reverse|Forward|Backward|Direction|convention|XNA|Unity" src tests README.md docs
```

## Recommended First Implementation Slice

Start with Phase 1 and the relevant active issues in `docs/feature-work/issue-tracker.md`. The guardrail tests should lock down the existing convention, while bug fixes such as `FixedQuaternion.FromDirection(Vector3d.Backward)` and `Vector3d.Direction` documentation/semantics should move through the issue tracker as separate implementation slices.

After that, Phase 2 can introduce the convention abstraction with much lower risk, because tests will already prove what must remain stable.

Avoid doing Phase 4 or Phase 5 in the same change as core fixes unless the adapter tests are ready. Engine adapters deserve their own review boundary.
